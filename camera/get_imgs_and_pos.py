import threading
import time
import rtde_receive
import rtde_control
import numpy as np
import cv2
import yaml
from pathlib import Path
from datetime import datetime
from queue import Queue
from pynput import keyboard

from dual_camera_wo_thread import DualCamera

# ====================
# 参数配置
# ====================
# 机器人参数
sensor_mass = 0.820  # 传感器质量，单位：千克
sensor_cog = [0.007, -0.006, 0.07]  # 质心位置，单位：米
robot_ip = "192.168.253.101"

# 相机参数（不再使用 OpenCV VideoCapture）
# 保留图像尺寸常量用于保存/处理（可按需修改）
IMG_WIDTH = 1600
IMG_HEIGHT = 1200

# 图像和位姿存储参数
NUM_POSES = 55  # 生成55个位姿
LC_IMG_DIR = "camera/lc_imgs/"  # 左相机图像文件夹
RC_IMG_DIR = "camera/rc_imgs/"  # 右相机图像文件夹
ROBOT_POS_FILE = "camera/robot_pos.txt"  # 机器人位姿文件
IMG_FORMAT = ".jpg"  # 图像格式

# ====================
# 全局变量
# ====================
rtde_c = None
rtde_r = None
quit_flag = False

# 存储数据
robot_poses = []  # 存储所有的4x4变换矩阵

# DualCamera 实例（在 initialize_cameras 中创建）
_dual_camera = None

# 多线程相关
thread_display = None
thread_robot = None
thread_keyboard = None

# 线程间通信
start_scan_event = threading.Event()    # 开始扫描的事件
stop_scan_event = threading.Event()     # 停止扫描的事件
display_stop_event = threading.Event()  # 停止显示线程的事件
robot_stop_event = threading.Event()    # 停止机器人线程的事件

# 扫描控制
FRAMES_PER_POSE = 10  # 每个位姿保存的帧数


# ====================
# 初始化函数
# ====================
def initialize_robot():
    """初始化机器人连接"""
    global rtde_c, rtde_r
    try:
        rtde_c = rtde_control.RTDEControlInterface(robot_ip)
        rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)
        
        # 设置 payload 和 CoG
        rtde_c.setPayload(sensor_mass, sensor_cog)
        print(f"[Robot] 机器人已连接: {robot_ip}")
        print(f"[Robot] Payload: {sensor_mass}, CoG: {sensor_cog}")
        return True
    except Exception as e:
        print(f"[Error] 机器人连接失败: {e}")
        return False


def initialize_cameras():
    """初始化左右相机"""
    global left_camera, right_camera, _dual_camera
    try:
        # 使用 DualCamera 类进行初始化
        _dual_camera = DualCamera(lc_imgs_path=LC_IMG_DIR, rc_imgs_path=RC_IMG_DIR, clear_on_init=False)
        ok = _dual_camera.initialize()
        if not ok:
            print("[Error] DualCamera 初始化失败")
            return False
        # left_camera/right_camera are not cv2.VideoCapture here, keep as None for compatibility
        left_camera = None
        right_camera = None
        print(f"[Camera] DualCamera 已初始化")
        return True
    except Exception as e:
        print(f"[Error] 相机初始化失败: {e}")
        return False


def create_image_directories():
    """创建图像存储目录"""
    try:
        Path(LC_IMG_DIR).mkdir(exist_ok=True)
        Path(RC_IMG_DIR).mkdir(exist_ok=True)
        print(f"[Setup] 图像目录已创建: {LC_IMG_DIR}, {RC_IMG_DIR}")
        return True
    except Exception as e:
        print(f"[Error] 创建目录失败: {e}")
        return False


# ====================
# 生成机器人位姿函数
# ====================
def generate_random_poses(num_poses=55):
    """
    生成55个随机的4x4变换矩阵位姿
    实际应用中可以根据需求修改这个函数
    
    :param num_poses: 位姿个数
    :return: 位姿列表，每个位姿是4x4的变换矩阵
    """
    poses = []
    
    # 获取当前位置作为基准位置
    try:
        current_pose = rtde_r.getActualTCPPose()  # 获取当前TCP位姿 [x, y, z, rx, ry, rz]
        print(f"[Pose] 当前位置: {current_pose}")
    except Exception as e:
        print(f"[Warning] 获取当前位置失败: {e}")
        current_pose = [0.3, 0.1, 0.2, 0, 0, 0]  # 默认位置
    
    # 生成55个位姿
    # 这里采用网格式生成：改变x和y坐标
    x_base, y_base, z_base = current_pose[0], current_pose[1], current_pose[2]
    rx_base, ry_base, rz_base = current_pose[3], current_pose[4], current_pose[5]
    
    # 创建一个5x11的网格（共55个点）
    x_offsets = np.linspace(-0.05, 0.05, 11)  # x方向，-5cm到+5cm
    y_offsets = np.linspace(-0.1, 0.1, 5)    # y方向，-10cm到+10cm
    
    idx = 0
    for y_offset in y_offsets:
        for x_offset in x_offsets:
            if idx >= num_poses:
                break
            
            # 创建4x4变换矩阵
            # 简单的方式：只改变位置，旋转保持不变
            pose_6d = [
                x_base + x_offset,
                y_base + y_offset,
                z_base,
                rx_base,
                ry_base,
                rz_base
            ]
            
            # 将6D位姿转换为4x4矩阵（这里简化处理）
            # 实际应用中应使用正确的旋转向量到旋转矩阵的转换
            pose_matrix = pose_6d_to_4x4(pose_6d)
            poses.append(pose_matrix)
            idx += 1
        
        if idx >= num_poses:
            break
    
    print(f"[Pose] 已生成 {len(poses)} 个位姿")
    return poses


def pose_6d_to_4x4(pose_6d):
    """
    将6D位姿转换为4x4变换矩阵
    
    :param pose_6d: 6D位姿 [x, y, z, rx, ry, rz]
    :return: 4x4变换矩阵
    """
    x, y, z, rx, ry, rz = pose_6d
    
    # 旋转向量到旋转矩阵的转换
    rvec = np.array([rx, ry, rz])
    R, _ = cv2.Rodrigues(rvec)
    
    # 创建4x4变换矩阵
    T = np.eye(4)
    T[0:3, 0:3] = R  # 旋转矩阵
    T[0:3, 3] = [x, y, z]  # 平移向量
    
    return T.flatten().tolist()  # 返回16元素列表


def pose_4x4_to_6d(pose_matrix):
    """
    将4x4矩阵转换回6D位姿（如需要）
    
    :param pose_matrix: 4x4变换矩阵（16元素列表）
    :return: 6D位姿 [x, y, z, rx, ry, rz]
    """
    T = np.array(pose_matrix).reshape(4, 4)
    
    # 提取平移向量
    position = T[0:3, 3].tolist()
    
    # 提取旋转矩阵并转换为旋转向量
    R = T[0:3, 0:3]
    rvec, _ = cv2.Rodrigues(R)
    rotation = rvec.flatten().tolist()
    
    return position + rotation


# ====================
# 相机控制函数
# ====================
def capture_images_from_both_cameras(pose_index):
    """
    从左右两个相机拍照
    
    :param pose_index: 位姿索引（0-54）
    :return: 是否拍照成功
    """
    try:
        # 仅使用 DualCamera 获取帧（不再使用 OpenCV VideoCapture）
        global _dual_camera
        if _dual_camera is None:
            print(f"[Error] DualCamera 未初始化，无法拍照（位姿 {pose_index}）")
            return False

        frame1, frame2 = _dual_camera.get_frames()
        if frame1 is None or frame2 is None:
            print(f"[Error] DualCamera 未获取到帧，位姿 {pose_index}")
            return False

        # 将 frame.buffer 转为 numpy image 并保存为要求的命名
        try:
            if getattr(frame1, 'buffer', None) is None or getattr(frame2, 'buffer', None) is None:
                print(f"[Error] 帧缓冲为空，位姿 {pose_index}")
                return False

            img_l = np.array(frame1.buffer, dtype=np.uint8).reshape(frame1.height, frame1.width, 4)
            img_r = np.array(frame2.buffer, dtype=np.uint8).reshape(frame2.height, frame2.width, 4)

            lc_filename = f"{LC_IMG_DIR}/{pose_index}{IMG_FORMAT}"
            rc_filename = f"{RC_IMG_DIR}/{pose_index}{IMG_FORMAT}"
            cv2.imwrite(lc_filename, img_l)
            cv2.imwrite(rc_filename, img_r)
            print(f"[Camera] 已保存图像 {pose_index}: {lc_filename}, {rc_filename}")
            return True
        except Exception as ex:
            print(f"[Error] 保存帧时出错: {ex}")
            return False
    except Exception as e:
        print(f"[Error] 拍照过程中出错: {e}")
        return False


# ====================
# 机器人控制函数
# ====================
def move_to_pose(pose_6d, velocity=0.5, acceleration=0.5):
    """
    移动机器人到指定的6D位姿
    
    :param pose_6d: 6D位姿 [x, y, z, rx, ry, rz]
    :param velocity: 运动速度
    :param acceleration: 加速度
    :return: 是否移动成功
    """
    try:
        # 使用moveL进行直线运动
        rtde_c.moveL(pose_6d, velocity, acceleration)
        print(f"[Robot] 已移动到位姿: {pose_6d}")
        return True
    except Exception as e:
        print(f"[Error] 机器人移动失败: {e}")
        return False


def move_to_pose_from_matrix(pose_matrix, velocity=0.5, acceleration=0.5):
    """
    从4x4矩阵提取6D位姿并移动机器人
    
    :param pose_matrix: 4x4变换矩阵
    :param velocity: 运动速度
    :param acceleration: 加速度
    :return: 是否移动成功
    """
    # 将4x4矩阵转换为6D位姿
    pose_6d = pose_4x4_to_6d(pose_matrix)
    return move_to_pose(pose_6d, velocity, acceleration)


# ====================
# 位姿存储函数
# ====================
def save_robot_poses_to_file(poses, filename=ROBOT_POS_FILE):
    """
    将所有机器人位姿保存到文件
    
    :param poses: 位姿列表，每个位姿是16元素列表（4x4矩阵展开）
    :param filename: 保存文件名
    """
    try:
        with open(filename, 'w') as f:
            for pose in poses:
                # 将16个元素写成一行，用空格分隔
                line = ' '.join([f"{x:.6f}" for x in pose]) + '\n'
                f.write(line)
        
        print(f"[File] 已保存 {len(poses)} 个位姿到文件: {filename}")
    except Exception as e:
        print(f"[Error] 保存位姿文件失败: {e}")


# ====================
# 多线程函数
# ====================
def thread_display_live_view():
    """
    线程1：实时显示左右相机的画面
    """
    print("[Thread-Display] 实时显示线程已启动")
    
    try:
        _dual_camera.display_live_view(duration=None)
    except Exception as e:
        print(f"[Thread-Display] 错误: {e}")
    finally:
        print("[Thread-Display] 实时显示线程已停止")
        display_stop_event.set()


def thread_keyboard_control():
    """
    线程2：监听键盘，按 's' 开始扫描，按 'q' 退出
    """
    print("[Thread-Keyboard] 键盘监听线程已启动")
    print("[Thread-Keyboard] 按 's' 开始扫描，按 'q' 退出")
    
    def on_press(key):
        try:
            # 按下 's' 键开始扫描
            if hasattr(key, 'char') and key.char == 's':
                print(f"\n[Thread-Keyboard] 用户按下 's'，开始扫描")
                start_scan_event.set()
            
            # 按下 'q' 键退出
            elif hasattr(key, 'char') and key.char == 'q':
                print(f"\n[Thread-Keyboard] 用户按下 'q'，停止扫描")
                stop_scan_event.set()
                display_stop_event.set()
                return False
        
        except AttributeError:
            pass
    
    try:
        with keyboard.Listener(on_press=on_press) as listener:
            while not stop_scan_event.is_set():
                time.sleep(0.1)
            listener.stop()
    
    except Exception as e:
        print(f"[Thread-Keyboard] 错误: {e}")
    finally:
        print("[Thread-Keyboard] 键盘监听线程已停止")
        stop_scan_event.set()


def automatic_scan_with_threads():
    """
    自动扫描：移动机器人到55个位姿，在每个位姿自动拍摄10张照片
    """
    global thread_display, thread_keyboard
    
    print("\n" + "="*50)
    print("自动扫描过程（多线程）")
    print("="*50)
    print("[Info] 控制说明:")
    print("       - 按 's' 键开始扫描")
    print("       - 按 'q' 键停止扫描和退出")
    print("="*50 + "\n")
    
    try:
        # 启动显示线程
        thread_display = threading.Thread(target=thread_display_live_view, daemon=True)
        thread_display.start()
        
        # 启动键盘监听线程
        thread_keyboard = threading.Thread(target=thread_keyboard_control, daemon=True)
        thread_keyboard.start()
        
        print("[Main] 等待用户按 's' 键开始扫描...")
        
        # 等待用户按 's' 键开始
        start_scan_event.wait()
        
        if stop_scan_event.is_set():
            print("[Main] 用户取消扫描")
            return False
        
        print("[Main] 扫描已开始！")
        
        # 生成位姿
        generated_poses = generate_random_poses(NUM_POSES)
        
        frame_global_index = 0  # 全局帧索引
        
        # 遍历每个位姿
        for pose_idx in range(NUM_POSES):
            if stop_scan_event.is_set():
                print("[Main] 用户停止扫描")
                break
            
            pose_matrix = generated_poses[pose_idx]
            pose_6d = pose_4x4_to_6d(pose_matrix)
            
            print(f"\n[Main] ========== 位姿 {pose_idx}/{NUM_POSES-1} ==========")
            print(f"[Main] 目标位姿: {[f'{x:.4f}' for x in pose_6d]}")
            
            # 移动机器人到该位姿
            print(f"[Main] 正在移动到位姿 {pose_idx}...")
            if not move_to_pose(pose_6d, velocity=0.2, acceleration=0.1):
                print(f"[Main] 位姿 {pose_idx} 移动失败，跳过")
                continue
            
            # 等待机器人到达稳定状态
            print(f"[Main] 等待机器人稳定（1秒）...")
            time.sleep(1.0)
            
            # 验证机器人是否到达目标位姿（可选，取决于精度要求）
            try:
                current_pose = rtde_r.getActualTCPPose()
                print(f"[Main] 当前位置: {[f'{x:.4f}' for x in current_pose]}")
            except:
                pass
            
            # 在该位姿自动保存 10 张照片
            print(f"[Main] 开始在位姿 {pose_idx} 保存 {FRAMES_PER_POSE} 张照片...")
            
            for frame_in_pose in range(FRAMES_PER_POSE):
                if stop_scan_event.is_set():
                    print("[Main] 用户停止扫描")
                    break
                
                # 获取当前帧
                frame1, frame2 = _dual_camera.get_frames()
                
                if frame1 is None or frame2 is None:
                    print(f"[Main] 位姿 {pose_idx} 第 {frame_in_pose+1} 帧获取失败")
                    continue
                
                # 保存帧
                try:
                    if getattr(frame1, 'buffer', None) is None or getattr(frame2, 'buffer', None) is None:
                        print(f"[Main] 位姿 {pose_idx} 第 {frame_in_pose+1} 帧缓冲为空")
                        continue
                    
                    img_l = np.array(frame1.buffer, dtype=np.uint8).reshape(frame1.height, frame1.width, 4)
                    img_r = np.array(frame2.buffer, dtype=np.uint8).reshape(frame2.height, frame2.width, 4)
                    
                    lc_filename = f"{LC_IMG_DIR}/{frame_global_index}{IMG_FORMAT}"
                    rc_filename = f"{RC_IMG_DIR}/{frame_global_index}{IMG_FORMAT}"
                    
                    cv2.imwrite(lc_filename, img_l)
                    cv2.imwrite(rc_filename, img_r)
                    
                    print(f"[Main] 位姿 {pose_idx} 已保存第 {frame_in_pose+1}/{FRAMES_PER_POSE} 张: {frame_global_index}")
                    
                    # 记录位姿（每个位姿只记录一次）
                    if frame_in_pose == 0:
                        try:
                            current_pose = rtde_r.getActualTCPPose()
                            pose_matrix_actual = pose_6d_to_4x4(current_pose)
                            robot_poses.append(pose_matrix_actual)
                            print(f"[Main] 已记录位姿 {pose_idx}: {[f'{x:.4f}' for x in current_pose]}")
                        except Exception as ex:
                            print(f"[Main] 记录位姿失败: {ex}")
                    
                    frame_global_index += 1
                
                except Exception as ex:
                    print(f"[Main] 保存帧时出错: {ex}")
                
                # 保存帧之间的间隔
                time.sleep(0.1)
            
            print(f"[Main] 位姿 {pose_idx} 完成，共保存 {FRAMES_PER_POSE} 张照片")
        
        print("\n[Main] 自动扫描完成")
        print(f"[Main] 成功采集 {len(robot_poses)} 个位姿，共 {frame_global_index} 张照片")
        
        # 保存所有位姿到文件
        if len(robot_poses) > 0:
            save_robot_poses_to_file(robot_poses)
        
        return True
    
    except Exception as e:
        print(f"[Error] 扫描过程出错: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    finally:
        # 确保所有事件都已设置
        display_stop_event.set()
        robot_stop_event.set()
        stop_scan_event.set()
        
        # 等待所有线程完成
        if thread_display and thread_display.is_alive():
            thread_display.join(timeout=5)
        if thread_keyboard and thread_keyboard.is_alive():
            thread_keyboard.join(timeout=5)
        
        print("[Main] 所有线程已停止")


# ====================
# 主扫描函数（已弃用，使用多线程版本 interactive_scan_with_threads）
# ====================
def scan_with_positions_legacy():
    """
    遗留的单线程扫描函数：移动机器人到55个位姿，拍照并记录位姿
    该函数已被 interactive_scan_with_threads 取代
    """
    global robot_poses, quit_flag
    
    print("\n" + "="*50)
    print("开始扫描过程")
    print("="*50)
    
    try:
        # 生成位姿
        generated_poses = generate_random_poses(NUM_POSES)
        
        # 遍历每个位姿
        for idx in range(NUM_POSES):
            if quit_flag:
                print("[Info] 扫描被用户中断")
                break
            
            pose_matrix = generated_poses[idx]
            pose_6d = pose_4x4_to_6d(pose_matrix)
            
            print(f"\n[Process] 处理位姿 {idx}/{NUM_POSES-1}")
            print(f"[Pose] 6D位姿: {[f'{x:.4f}' for x in pose_6d]}")
            
            # 移动机器人到该位姿
            if not move_to_pose(pose_6d, velocity=0.2, acceleration=0.1):
                print(f"[Warning] 跳过位姿 {idx}")
                continue
            
            # 等待机器人稳定
            time.sleep(1.0)
            
            # 拍照
            if not capture_images_from_both_cameras(idx):
                print(f"[Warning] 位姿 {idx} 拍照失败")
                continue
            
            # 记录位姿（保存4x4矩阵的16个元素）
            robot_poses.append(pose_matrix)
            
            print(f"[Progress] 完成位姿 {idx}/{NUM_POSES-1}")
        
        print("\n[Success] 扫描过程完成")
        print(f"[Info] 成功采集 {len(robot_poses)} 个位姿")
        
        # 保存所有位姿到文件
        save_robot_poses_to_file(robot_poses)
        
        return True
    except Exception as e:
        print(f"[Error] 扫描过程出错: {e}")
        return False


# ====================
# 清理函数
# ====================
def cleanup():
    """清理资源"""
    try:
        print("\n[Cleanup] 正在清理资源...")
        
        # 关闭 DualCamera（如果存在）
        if '_dual_camera' in globals() and _dual_camera is not None:
            try:
                _dual_camera.close()
            except Exception:
                pass

        # 停止机器人
        if rtde_c is not None:
            rtde_c.servoStop()
            rtde_c.stopScript()
            rtde_c.disconnect()

        if rtde_r is not None:
            rtde_r.disconnect()

        print("[Cleanup] 资源已清理完成")
    except Exception as e:
        print(f"[Error] 清理过程出错: {e}")


# ====================
# 主函数
# ====================
def main():
    """主函数"""
    global quit_flag
    
    print("\n" + "="*50)
    print("机器人超声扫描系统 - 图像采集和位姿记录")
    print("="*50)
    print(f"[Config] 扫描位姿数: {NUM_POSES}")
    print(f"[Config] 每个位姿保存帧数: {FRAMES_PER_POSE}")
    print(f"[Config] 总预期照片数: {NUM_POSES * FRAMES_PER_POSE}")
    print(f"[Config] 左相机图像目录: {LC_IMG_DIR}")
    print(f"[Config] 右相机图像目录: {RC_IMG_DIR}")
    print(f"[Config] 位姿保存文件: {ROBOT_POS_FILE}")
    print("="*50 + "\n")
    
    try:
        # 初始化
        if not create_image_directories():
            return False
        
        if not initialize_robot():
            return False
        
        if not initialize_cameras():
            return False
        
        # 提示用户确认
        print("[Info] 系统已初始化完成")
        print("[Info] 按 Enter 键继续，或按 Ctrl+C 退出...")
        input()
        
        # 执行自动扫描（使用多线程）
        automatic_scan_with_threads()
        
        return True
    except KeyboardInterrupt:
        print("\n[Info] 用户中断")
        quit_flag = True
        display_stop_event.set()
        robot_stop_event.set()
        stop_scan_event.set()
        return False
    except Exception as e:
        print(f"[Error] 程序出错: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        cleanup()


if __name__ == "__main__":
    main()
