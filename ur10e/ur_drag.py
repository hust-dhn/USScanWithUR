import threading
import time
import sys
import enum
from pynput import keyboard
import rtde_control
import rtde_receive
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

import numpy as np
from scipy.spatial.transform import Rotation as R

import yaml
import os
import math 

from camera.ur10e_kine import UR10eKine

# ====================
# 配置与常量
# ====================
ROBOT_IP = "192.168.253.101"
CONTROL_HZ = 125
DT = 1.0 / CONTROL_HZ

# 传感器负载参数
SENSOR_MASS = 0.820  # 单位: kg
SENSOR_COG = [0.007, -0.006, 0.07]  # 单位: m

# 自由拖动参数
FREE_AXES = [1, 1, 1, 1, 1, 1]  # 1=启用, 0=禁用
FREE_FEATURE = [1, 1, 1, 0.5, 0.5, 0.5]  # 阻尼/权重

# 伺服控制参数
SERVO_MOVEL_PARAMS = {
    "velocity": 0.5,
    "acceleration": 0.5,
    "lookahead_time": 0.1,  # 建议范围 0.03 ~ 0.2
    "gain": 300
}

def load_calibration_matrix(yaml_file_path):
    """
    读取 YAML 文件并将变换列表转换为 4x4 矩阵
    """
    if not os.path.exists(yaml_file_path):
        print(f"[ERROR] 找不到文件: {yaml_file_path}")
        return np.eye(4)

    try:
        with open(yaml_file_path, 'r') as f:
            data = yaml.safe_load(f)
            
        # 1. 检查变换键是否存在
        if 'transform' not in data:
            print("[ERROR] 配置文件中缺少 'transform' 键")
            return np.eye(4)
            
        transform_list = data['transform']
        
        # 2. 转换为 numpy 数组 (按行平铺)
        T_sen22tcp = np.array(transform_list).reshape(4, 4)
        
        print("[INFO] 标定矩阵加载成功")
        return T_sen22tcp

    except Exception as e:
        print(f"[ERROR] 解析配置文件失败: {e}")
        return np.eye(4)

class RobotMode(enum.Enum):
    """机器人运行模式枚举"""
    IDLE = 0        # 空闲状态
    FREEDRIVE = 1   # 自由拖动模式
    SCAN = 2        # 扫描模式
    SERVO = 3       # 伺服控制模式

class PoseSubscriber(Node):
    """ROS 2 订阅节点，用于接收位姿配准结果"""
    def __init__(self, controller_instance):
        super().__init__('pose_subscriber_node')
        self.controller = controller_instance # 引用控制器实例
        self.subscription = self.create_subscription(
            PoseStamped,
            '/registration_result',
            self.listener_callback,
            10
        )
        print("[ROS] 节点已启动，正在监听配准话题")

    def listener_callback(self, msg):
        # 1. 提取位置信息
        t = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        
        # 2. 提取四元数
        quat = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]

        # 3. 构建 4x4 变换矩阵
        T = np.eye(4)
        r = R.from_quat(quat)
        T[:3, :3] = r.as_matrix()
        T[:3, 3] = t
        
        # 4. 更新控制器中的最新配准位姿
        self.controller.latest_registration_pos_ros = T


class URRobotController:
    def __init__(self, ip):
        """
        初始化控制器并建立通讯连接
        """
        # 存储最新的配准位姿
        self.latest_registration_pos_ros = None
        # load best_transform.txt
        self.latest_registration_pose = np.loadtxt("/home/zzz/ros2_ws/src/Camera_example/CPP/app/PCLProject/debug_pcd/best_transform.txt").reshape(4,4)
        print(f"[INFO] 初始配准矩阵:\n{self.latest_registration_pose}")
        # 初始化 ROS 2
        print("[SYSTEM] 正在初始化 ROS 环境")
        rclpy.init(args=None)
        self.ros_node = PoseSubscriber(self)
        
        # 连接机器人
        print(f"[ROBOT] 正在连接至机器人: {ip}")
        try:
            self.ur10e_kine = UR10eKine(ip)
            # self.rtde_c = rtde_control.RTDEControlInterface(ip)
            # self.rtde_r = rtde_receive.RTDEReceiveInterface(ip)
            self.rtde_c = self.ur10e_kine.rtde_c
            self.rtde_r = self.ur10e_kine.rtde_r
            print("[SUCCESS] 机器人连接成功")
        except Exception as e:
            print(f"[ERROR] 连接机器人失败: {e}")
            sys.exit(1)

        # 设置机器人负载
        self.rtde_c.setPayload(SENSOR_MASS, SENSOR_COG)
        print(f"[INFO] 负载参数设置完毕: 质量 {SENSOR_MASS}kg")

        # 状态控制变量
        self.current_mode = RobotMode.IDLE
        self.quit_event = threading.Event()
        self.target_pose = None
        self._last_mode = RobotMode.IDLE
        self.scan_idle_bool = False

        # 加载轨迹文件
        self.traj_file_path = "point_cloud/path_matrix.txt"
        print(f"[FILE] 正在加载轨迹文件: {self.traj_file_path}")
        try:
            self.trajectory_data = np.loadtxt(self.traj_file_path)
            self.traj_length = len(self.trajectory_data)
            print(f"[SUCCESS] 轨迹加载完成，总点数: {self.traj_length}")
        except Exception as e:
            print(f"[ERROR] 无法加载轨迹文件: {e}")
            self.trajectory_data = None
            self.traj_length = 0
        
        self.count = 1
        self.T_sen22tcp = load_calibration_matrix("camera/cfg/T_cam2tcp.yaml")
        self.T_sen02sen2 = load_calibration_matrix("camera/cfg/T_sensor0_to_sensor2.yaml")
        self.T_end2tcp = load_calibration_matrix("ur10e/cfg/T_end2tcp.yaml")
        self.scan_success_bool = False



    def start(self):
        """启动控制循环、ROS线程和键盘监听"""
        
        # 1. 启动控制循环线程
        self.control_thread = threading.Thread(target=self._control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()

        # 2. 启动 ROS 2 监听线程
        self.ros_thread = threading.Thread(target=self._ros_spin_loop)
        self.ros_thread.daemon = True
        self.ros_thread.start()

        # 3. 启动键盘监听
        print("[SYSTEM] 系统准备就绪，按 'h' 查看操作指令")
        self._listen_keyboard()

    def stop(self):
        """清理并安全退出系统"""
        print("[SYSTEM] 正在关闭系统")
        self.quit_event.set()
        
        if rclpy.ok():
            self.ros_node.destroy_node()
            rclpy.shutdown()
            print("[ROS] 节点已正常关闭")

        try:
            if self.rtde_c.isConnected():
                self.rtde_c.endFreedriveMode()
                self.rtde_c.stopScript()
                self.rtde_c.disconnect()
                self.rtde_r.disconnect()
                print("[ROBOT] 机器人连接已断开")
        except Exception as e:
            print(f"[ERROR] 退出通讯时发生错误: {e}")
        
        print("[INFO] 程序已安全结束")

    def _ros_spin_loop(self):
        """处理 ROS 事件"""
        try:
            rclpy.spin(self.ros_node)
        except Exception:
            pass

    def _control_loop(self):
        """主控制循环线程"""
        while not self.quit_event.is_set():
            start_time = time.time()

            # 处理模式切换
            if self.current_mode != self._last_mode:
                self._handle_mode_change(self._last_mode, self.current_mode)
                self._last_mode = self.current_mode

            # 执行当前模式行为
            if self.current_mode == RobotMode.SERVO:
                if self.target_pose:
                    self.rtde_c.servoL(
                        self.target_pose,
                        SERVO_MOVEL_PARAMS["velocity"],
                        SERVO_MOVEL_PARAMS["acceleration"],
                        DT,
                        SERVO_MOVEL_PARAMS["lookahead_time"],
                        SERVO_MOVEL_PARAMS["gain"]
                    )
                else:
                    print("[WARN] 伺服模式缺少目标位置，切换至空闲")
                    self.current_mode = RobotMode.IDLE
            
            elif self.current_mode == RobotMode.SCAN:
                self.scan_mode_loop()

            # 维持固定频率
            end_time = time.time()
            duration = end_time - start_time
            if duration < DT:
                time.sleep(DT - duration)

    def _handle_mode_change(self, old_mode, new_mode):
        """处理切换模式时的状态机逻辑"""
        
        # 退出行为
        if old_mode == RobotMode.FREEDRIVE:
            self.rtde_c.endFreedriveMode()
            print("[MODE] 退出自由拖动模式")
        
        # 进入行为
        if new_mode == RobotMode.FREEDRIVE:
            self.rtde_c.freedriveMode(FREE_AXES, FREE_FEATURE)
            print("[MODE] 进入自由拖动模式")
        
        elif new_mode == RobotMode.SERVO:
            self.target_pose = self.rtde_r.getActualTCPPose()
            print(f"[MODE] 进入伺服控制模式，当前锁定位置: {self.target_pose[:3]}")

        elif new_mode == RobotMode.IDLE:
            if old_mode == RobotMode.SERVO:
                self.rtde_c.servoStop()
            print("[MODE] 进入空闲模式")

        elif new_mode == RobotMode.SCAN:
            print("[MODE] 进入超声扫描模式")
            if self.latest_registration_pose is not None:
                print(f"[INFO] 已获取配准数据")
            else:
                print("[WARN] 暂无配准数据，等待输入")
            
            if old_mode == RobotMode.SERVO:
                self.rtde_c.servoStop()


    def get_trajectory_pose(self, count):
        """根据计数索引获取轨迹矩阵"""
        if self.trajectory_data is None:
            print("[ERROR] 轨迹数据为空")
            return np.eye(4)
            
        if 0 <= count < self.traj_length:
            flat_pose = self.trajectory_data[count]
            return flat_pose.reshape(4, 4)
        else:
            print(f"[ERROR] 索引 {count} 溢出")
            return np.eye(4)

    def Tmatrix_to_XYZRXRYRZ(self, T_matrix):
        """矩阵转位姿向量"""
        x = T_matrix[0, 3]
        y = T_matrix[1, 3]
        z = T_matrix[2, 3]
        R_mat = T_matrix[:3, :3]
        trace = R_mat[0, 0] + R_mat[1, 1] + R_mat[2, 2]
        cos_angle = (trace - 1) / 2
        cos_angle = max(-1.0, min(1.0, cos_angle)) # 防止溢出
        angle = math.acos(cos_angle) 
        if abs(angle) < 1e-6:
            return [x, y, z, 0.0, 0.0, 0.0]
        sin_angle = math.sin(angle) 
        factor = 1 / (2 * sin_angle)
        rx = factor * (R_mat[2, 1] - R_mat[1, 2]) * angle
        ry = factor * (R_mat[0, 2] - R_mat[2, 0]) * angle
        rz = factor * (R_mat[1, 0] - R_mat[0, 1]) * angle
        return [x, y, z, rx, ry, rz]

    def XYZRXRYRZ_to_Tmatrix(self, xyzrxryrz):
        """位姿向量转矩阵"""
        x, y, z, rx, ry, rz = xyzrxryrz
        angle = math.sqrt(rx**2 + ry**2 + rz**2)
        if angle < 1e-6:
            T = np.eye(4)
            T[0,3], T[1,3], T[2,3] = x, y, z
            return T
        ux, uy, uz = rx / angle, ry / angle, rz / angle
        cos_a, sin_a = math.cos(angle), math.sin(angle)
        omc = 1 - cos_a
        R_mat = np.array([
            [cos_a + ux**2 * omc, ux * uy * omc - uz * sin_a, ux * uz * omc + uy * sin_a],
            [uy * ux * omc + uz * sin_a, cos_a + uy**2 * omc, uy * uz * omc - ux * sin_a],
            [uz * ux * omc - uy * sin_a, uz * uy * omc + ux * sin_a, cos_a + uz**2 * omc]
        ])
        T = np.eye(4)
        T[:3, :3] = R_mat
        T[:3, 3] = [x, y, z]
        return T

    def save_pose_to_file(self, pose, filename):
        """
        将单个位姿数据追加到文件末尾
        pose: 包含6个值的列表或元组 [x, y, z, rx, ry, rz]
        """
        with open(filename, 'a') as f:  # 'a' 表示追加模式
            # 将位姿数据转换为字符串并写入
            line = f"{pose[0]:.6f} {pose[1]:.6f} {pose[2]:.6f} "
            line += f"{pose[3]:.6f} {pose[4]:.6f} {pose[5]:.6f}\n"
            f.write(line)
        
        print(f"位姿已追加到 {filename}")

    def scan_mode_loop(self):
        """超声扫描模式执行逻辑""" 
        q = self.rtde_r.getActualQ()
        last_q = q.copy()
        while not self.scan_idle_bool:
            if self.count >= self.traj_length:
                print("[SCAN] 扫描已结束，返回空闲状态")
                self.current_mode = RobotMode.IDLE
                self.count = 1
                self.scan_idle_bool = True
                return
            
            T_sen22tcp = self.T_sen22tcp
            T_sen02sen2 = self.T_sen02sen2

            print(f"[SCAN] 当前采样点索引: {self.count}")
            T_traj2global = self.get_trajectory_pose(self.count)
            print(f"[SCAN] 目标轨迹矩阵:\n{T_traj2global}")
            if self.latest_registration_pose is not None:
                # 简化逻辑，此处假设矩阵运算已定义
                T_latest_inv = np.linalg.inv(self.latest_registration_pose)
                # print(f"[SCAN] 最新配准矩阵:\n{self.latest_registration_pose}")
                T_traj2realtime = T_latest_inv @ T_traj2global
                T_traj2tcp = T_sen22tcp @ T_sen02sen2 @ T_traj2realtime
                T_tcp2base_current = self.ur10e_kine.FK(q)
                # T_tcp2base_current = self.XYZRXRYRZ_to_Tmatrix(self.rtde_r.getActualTCPPose())
                T_traj2base = T_tcp2base_current @ T_traj2tcp

                T_tcp2end = np.linalg.inv(self.T_end2tcp)
                T_tcp2base_target = T_traj2base @ T_tcp2end

                target_joint = self.ur10e_kine.IK(T_tcp2base_target, last_q)
                print(f"[SCAN] 目标关节角度: {[round(angle,4) for angle in target_joint]}")
                self.rtde_c.moveJ(target_joint, 0.2, 0.3)
                print(f"[SCAN] 机器人已移动至采样点: {self.count}")
                last_q = target_joint

                # scan_pose = self.Tmatrix_to_XYZRXRYRZ(T_tcp2base_target)
                self.save_pose_to_file(target_joint, 'ur10e/cfg/joints.txt')

                # scan_pose_1 = self.Tmatrix_to_XYZRXRYRZ(T_traj2base)
                # self.save_pose_to_file(scan_pose_1, 'ur10e/cfg/poses_traj.txt')
                # print(f"[SCAN] 计算目标位姿: {scan_pose}")

                # 首尾点精细扫描逻辑
                # if self.count == 1 or self.count == self.traj_length - 1:
                #     print(f"[SCAN] 到达关键采样点: {self.count}")
                #     # self.rtde_c.moveL(scan_pose, 0.1, 0.2)
                #     # self.rtde_c.moveJ(target_joint, 0.1, 0.2)
                #     print("[SCAN] 开始停留等待")
                #     while not self.scan_success_bool:
                #         time.sleep(0.001)
                #     print("[SCAN] 停留结束，继续任务")
                # else:
                #     # self.rtde_c.moveJ(target_joint, 0.2, 0.3)
                #     print(f"[SCAN] 移动至采样点: {self.count}")
                #     # self.rtde_c.servoL(scan_pose, 0.5, 0.5, 0.02, 0.1, 300)
                #     # self.rtde_c.moveL(scan_pose, 0.1, 0.2)
                #     pass

                self.count += 1
                self.scan_success_bool = False
                
            # if self.count % 50 == 0: 
            print(f"[FOLLOW] 任务进度: {self.count}/{self.traj_length}")

    def _on_key_press(self, key):
        """键盘操作响应"""
        try:
            if hasattr(key, 'char'):
                if key.char == 'd':
                    self.current_mode = RobotMode.FREEDRIVE
                elif key.char == 'c':
                    self.current_mode = RobotMode.IDLE
                elif key.char == 'u':
                    self.scan_success_bool = False
                    self.scan_idle_bool = False
                    self.current_mode = RobotMode.SCAN 
                elif key.char == 't':
                    self.current_mode = RobotMode.SERVO
                elif key.char == 'p':
                    q = self.rtde_r.getActualQ()
                    tcp = self.rtde_r.getActualTCPPose()
                    print(f"[PRINT] 关节数据: {[round(x, 4) for x in q]}")
                    print(f"[PRINT] 笛卡尔坐标: {[round(x, 4) for x in tcp]}")
                    print(f"[PRINT] 配准状态: {'已加载' if self.latest_registration_pose is not None else '未接收'}")
                elif key.char == 'h':
                    self._print_help()
                elif key.char == 's':
                    if self.current_mode == RobotMode.SCAN:
                        self.scan_success_bool = True
                elif key.char == 'q':
                    return False
                elif key.char == 'r':
                    self.latest_registration_pose = self.latest_registration_pos_ros
        except AttributeError:
            pass

    def _listen_keyboard(self):
        """阻塞式监听键盘"""
        with keyboard.Listener(on_press=self._on_key_press) as listener:
            listener.join()
        self.stop()

    def _print_help(self):
        """显示操作手册"""
        print("[HELP] [d]: 启用自由拖动")
        print("[HELP] [c]: 停止当前动作")
        print("[HELP] [t]: 启用伺服控制")
        print("[HELP] [u]: 开启扫描任务")
        print("[HELP] [p]: 打印系统状态")
        print("[HELP] [q]: 安全退出系统")
        print("[HELP] [h]: 显示帮助信息")
        print("[HELP] [s]: 扫描模式下确认采样完成")

# ====================
# 程序入口
# ====================
def main():
    robot = URRobotController(ROBOT_IP)
    robot.start()

if __name__ == "__main__":
    main()