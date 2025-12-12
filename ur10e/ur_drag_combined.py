import threading
import time
import rtde_receive
import rtde_control
import numpy as np
import cv2
from pynput import keyboard
from typing import List


# ===== User-editable defaults =====
ROBOT_IP = "192.168.253.101"
SENSOR_MASS = 0.82
SENSOR_COG = [0.007, -0.006, 0.07]
FREE_AXES = [1, 1, 1, 1, 1, 1]
FEATURE = [1, 1, 1, 0.5, 0.5, 0.5]
TOOL_OFFSET = [0.0, -0.07625, 0.07708, 0.0, 0.0, 0.0]
HZ = 125
SERVO_V = 0.5
SERVO_A = 0.5
SERVO_GAIN = 300
MOVE_DELTA = 0.05
# ================================


class URDrag:
    """
    Consolidated, flexible UR drag + control helper.
    Usage:
      d: toggle freedrive
      c: stop freedrive
      t: set custom target (input) and start servoing
      p: set current pose as target
      o: print tool pose
      e: print end-effector pose
      u/i/j/l/k/m: move target +/-0.05m in axes
      1: test tool calibration
      2: simple tool calibration (4-point)
      q: quit
    """

    def __init__(self,
                 robot_ip: str = None,
                 sensor_mass: float = None,
                 sensor_cog: List[float] = None,
                 free_axes: List[int] = None,
                 feature: List[float] = None,
                 tool_offset: List[float] = None,
                 hz: int = None,
                 servo_v: float = None,
                 servo_a: float = None,
                 servo_gain: int = None,
                 move_delta: float = None):
        # use module-level defaults when None provided (easy to edit at file top)
        self.robot_ip = robot_ip if robot_ip is not None else ROBOT_IP
        self.sensor_mass = sensor_mass if sensor_mass is not None else SENSOR_MASS
        self.sensor_cog = sensor_cog if sensor_cog is not None else SENSOR_COG
        self.free_axes = free_axes if free_axes is not None else FREE_AXES
        self.feature = feature if feature is not None else FEATURE
        # tool_offset: [dx, dy, dz, drx, dry, drz] (meters, rotation vector)
        self.tool_offset = tool_offset if tool_offset is not None else TOOL_OFFSET

        self.hz = hz if hz is not None else HZ
        self.dt = 1.0 / float(self.hz)
        self.lookahead_time = 5 * self.dt

        # servo parameters
        self.servo_v = servo_v if servo_v is not None else SERVO_V
        self.servo_a = servo_a if servo_a is not None else SERVO_A
        self.servo_gain = servo_gain if servo_gain is not None else SERVO_GAIN
        self.move_delta = move_delta if move_delta is not None else MOVE_DELTA

        # RTDE interfaces (initialized in start)
        self.rtde_c = None
        self.rtde_r = None

        # control flags
        self.quit_flag = False
        self.freedrive_enabled = False
        self.freedrive_active = False  # whether freedriveMode() currently active
        self.auto_callback_flag = False

        self.target_pose = None  # 6d target [x,y,z,rx,ry,rz]

        # threads
        self.threads = []

    # ------------------- utils -------------------
    def pose_to_matrix(self, pose):
        """pose: [x,y,z,rx,ry,rz] where rx,ry,rz is rotation vector (Rodrigues)"""
        pos = np.array(pose[:3], dtype=float).reshape(3)
        rvec = np.array(pose[3:], dtype=float).reshape(3)
        Rmat, _ = cv2.Rodrigues(rvec)
        T = np.eye(4, dtype=float)
        T[:3, :3] = Rmat
        T[:3, 3] = pos
        return T

    def matrix_to_pose(self, T):
        Rmat = T[:3, :3]
        rvec, _ = cv2.Rodrigues(Rmat)
        pos = T[:3, 3]
        return np.concatenate([pos, rvec.flatten()]).tolist()

    def end_to_tool_transform(self, end_pose, tool_offset=None):
        """T_tool = T_end * T_tool_rel"""
        to = tool_offset if tool_offset is not None else self.tool_offset
        T_end = self.pose_to_matrix(end_pose)
        T_tool_rel = self.pose_to_matrix(to)
        T_tool = T_end @ T_tool_rel
        return self.matrix_to_pose(T_tool)

    def tool_to_end_transform(self, tool_pose, tool_offset=None):
        to = tool_offset if tool_offset is not None else self.tool_offset
        T_tool = self.pose_to_matrix(tool_pose)
        T_tool_rel = self.pose_to_matrix(to)
        T_end = T_tool @ np.linalg.inv(T_tool_rel)
        return self.matrix_to_pose(T_end)

    # ------------------- robot/io -------------------
    def connect(self):
        self.rtde_c = rtde_control.RTDEControlInterface(self.robot_ip)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(self.robot_ip)
        try:
            self.rtde_c.setPayload(self.sensor_mass, self.sensor_cog)
        except Exception:
            # some firmwares may not accept repeated setPayload calls
            pass

    def disconnect(self):
        try:
            if self.rtde_c is not None:
                self.rtde_c.servoStop()
                self.rtde_c.stopScript()
                self.rtde_c.disconnect()
        except Exception:
            pass
        try:
            if self.rtde_r is not None:
                self.rtde_r.disconnect()
        except Exception:
            pass

    # ------------------- behaviors -------------------
    def set_target_to_current(self):
        cur = self.rtde_r.getActualTCPPose()
        self.target_pose = cur.copy()
        return self.target_pose

    def set_custom_target(self, pose6d):
        if len(pose6d) != 6:
            return False
        self.target_pose = pose6d.copy()
        return True

    def move_target_delta(self, delta):
        if self.target_pose is None:
            return False
        for i in range(6):
            self.target_pose[i] += delta[i]
        return True

    def print_positions(self):
        print("End-effector:", self.rtde_r.getActualTCPPose())
        print("Tool pose:", self.end_to_tool_transform(self.rtde_r.getActualTCPPose()))

    def test_tool_calibration(self):
        print("Testing tool calibration...")
        end = self.rtde_r.getActualTCPPose()
        tool = self.end_to_tool_transform(end)
        end_from_tool = self.tool_to_end_transform(tool)
        print("end:", end)
        print("tool:", tool)
        print("end_from_tool:", end_from_tool)

    def simple_tool_calibration(self):
        print("Simple 4-point tool calibration: collect 4 poses")
        poses = []
        for i in range(4):
            input(f"Move to pose {i+1} and press Enter...")
            poses.append(self.rtde_r.getActualTCPPose())
            print("Recorded:", poses[-1])
        return poses

    # ------------------- threads -------------------
    def freedrive_thread(self):
        print("Freedrive thread started")
        while not self.quit_flag:
            try:
                if self.freedrive_enabled and not self.freedrive_active:
                    try:
                        self.rtde_c.freedriveMode(self.free_axes, self.feature)
                        self.freedrive_active = True
                    except Exception as e:
                        print("Failed to start freedriveMode:", e)
                elif not self.freedrive_enabled and self.freedrive_active:
                    try:
                        self.rtde_c.endFreedriveMode()
                    except Exception:
                        pass
                    self.freedrive_active = False
            except Exception as e:
                print("Freedrive loop error:", e)
            time.sleep(0.05)
        # ensure end
        try:
            self.rtde_c.endFreedriveMode()
        except Exception:
            pass
        print("Freedrive thread exiting")

    def auto_callback_thread(self):
        print("Auto callback thread started")
        while not self.quit_flag:
            if self.auto_callback_flag and self.target_pose is not None:
                try:
                    # use servoL for smooth cartesian control
                    self.rtde_c.servoL(self.target_pose, self.servo_v, self.servo_a, self.hz, self.lookahead_time, self.servo_gain)
                except Exception as e:
                    print("servoL error:", e)
                    self.auto_callback_flag = False
            time.sleep(self.dt)
        print("Auto callback thread exiting")

    def keyboard_thread(self):
        print("Keyboard listener started")

        def on_press(key):
            try:
                if hasattr(key, 'char') and key.char == 'd':
                    self.freedrive_enabled = True
                    print("Freedrive enabled")

                elif hasattr(key, 'char') and key.char == 'c':
                    self.freedrive_enabled = False
                    print("Freedrive disabled")

                elif hasattr(key, 'char') and key.char == 't':
                    # set custom target via input
                    s = input("Enter target x y z rx ry rz: ")
                    vals = s.split()
                    if len(vals) == 6:
                        vals = [float(v) for v in vals]
                        self.set_custom_target(vals)
                        self.auto_callback_flag = True
                        print("Custom target set and servoing started")
                    else:
                        print("Invalid input")

                elif hasattr(key, 'char') and key.char == 'p':
                    self.set_target_to_current()
                    print("Target set to current pose")

                elif hasattr(key, 'char') and key.char == 'o':
                    print("Tool:", self.end_to_tool_transform(self.rtde_r.getActualTCPPose()))

                elif hasattr(key, 'char') and key.char == 'e':
                    print("End effector:", self.rtde_r.getActualTCPPose())

                elif hasattr(key, 'char') and key.char == '1':
                    self.test_tool_calibration()

                elif hasattr(key, 'char') and key.char == '2':
                    self.simple_tool_calibration()

                elif hasattr(key, 'char') and key.char == 'q':
                    print("Quit requested")
                    self.quit_flag = True
                    return False

                elif hasattr(key, 'char') and key.char in ['u', 'i', 'j', 'l', 'k', 'm']:
                    # map to deltas (u:+z, i:-z, j:-x, l:+x, k:+y, m:-y)
                    md = self.move_delta
                    map_d = {
                        'u': [0, 0, md, 0, 0, 0],
                        'i': [0, 0, -md, 0, 0, 0],
                        'j': [-md, 0, 0, 0, 0, 0],
                        'l': [md, 0, 0, 0, 0, 0],
                        'k': [0, md, 0, 0, 0, 0],
                        'm': [0, -md, 0, 0, 0, 0],
                    }
                    self.move_target_delta(map_d[key.char])
                    print("Target moved by", map_d[key.char], "New target:", self.target_pose)

            except AttributeError:
                pass

        with keyboard.Listener(on_press=on_press) as listener:
            listener.join()
        print("Keyboard listener exiting")

    # ------------------- lifecycle -------------------
    def start(self):
        print("Starting URDrag...")
        self.connect()
        # spawn threads
        t_kb = threading.Thread(target=self.keyboard_thread, daemon=True)
        t_fd = threading.Thread(target=self.freedrive_thread, daemon=True)
        t_ac = threading.Thread(target=self.auto_callback_thread, daemon=True)
        self.threads = [t_kb, t_fd, t_ac]
        for t in self.threads:
            t.start()

        try:
            while not self.quit_flag:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("Interrupted by user")
            self.quit_flag = True

        # wait for threads to finish
        for t in self.threads:
            t.join(timeout=1.0)
        self.disconnect()
        print("URDrag stopped")


if __name__ == '__main__':
    drag = URDrag()
    drag.start()
