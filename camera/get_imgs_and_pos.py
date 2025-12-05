import threading
import time
import rtde_receive
import rtde_control
import numpy as np
import cv2
import yaml
from pathlib import Path
from datetime import datetime

# ====================
# 参数配置
# ====================
# 机器人参数
sensor_mass = 0.820  # 传感器质量，单位：千克
sensor_cog = [0.007, -0.006, 0.07]  # 质心位置，单位：米
robot_ip = "192.168.253.101"

# 相机参数
CAMERA_LC_INDEX = 0  # 左相机编号
CAMERA_RC_INDEX = 1  # 右相机编号
IMG_WIDTH = 1280
IMG_HEIGHT = 960

# 图像和位姿存储参数
NUM_POSES = 55  # 生成55个位姿
LC_IMG_DIR = "lc_imgs"  # 左相机图像文件夹
RC_IMG_DIR = "rc_imgs"  # 右相机图像文件夹
ROBOT_POS_FILE = "robot_pos.txt"  # 机器人位姿文件
IMG_FORMAT = ".jpg"  # 图像格式

# ====================
# 全局变量
# ====================
rtde_c = None
rtde_r = None
quit_flag = False

# 存储数据
robot_poses = []  # 存储所有的4x4变换矩阵
left_camera = None
right_camera = None


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
    global left_camera, right_camera
    try:
        left_camera = cv2.VideoCapture(CAMERA_LC_INDEX)
        right_camera = cv2.VideoCapture(CAMERA_RC_INDEX)
        
        # 设置相机分辨率
        left_camera.set(cv2.CAP_PROP_FRAME_WIDTH, IMG_WIDTH)
        left_camera.set(cv2.CAP_PROP_FRAME_HEIGHT, IMG_HEIGHT)
        right_camera.set(cv2.CAP_PROP_FRAME_WIDTH, IMG_WIDTH)
        right_camera.set(cv2.CAP_PROP_FRAME_HEIGHT, IMG_HEIGHT)
        
        # 检查相机是否成功打开
        if not left_camera.isOpened() or not right_camera.isOpened():
            print("[Error] 无法打开相机")
            return False
        
        print(f"[Camera] 相机已初始化")
        print(f"[Camera] 分辨率: {IMG_WIDTH}x{IMG_HEIGHT}")
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
        # 拍摄左相机
        ret_lc, frame_lc = left_camera.read()
        if not ret_lc or frame_lc is None:
            print(f"[Error] 左相机拍摄失败，位姿 {pose_index}")
            return False
        
        # 拍摄右相机
        ret_rc, frame_rc = right_camera.read()
        if not ret_rc or frame_rc is None:
            print(f"[Error] 右相机拍摄失败，位姿 {pose_index}")
            return False
        
        # 保存左相机图像
        lc_filename = f"{LC_IMG_DIR}/{pose_index}{IMG_FORMAT}"
        cv2.imwrite(lc_filename, frame_lc)
        
        # 保存右相机图像
        rc_filename = f"{RC_IMG_DIR}/{pose_index}{IMG_FORMAT}"
        cv2.imwrite(rc_filename, frame_rc)
        
        print(f"[Camera] 已保存图像 {pose_index}: {lc_filename}, {rc_filename}")
        return True
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
# 主扫描函数
# ====================
def scan_with_positions():
    """
    主扫描函数：移动机器人到55个位姿，拍照并记录位姿
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
        
        # 释放相机
        if left_camera is not None:
            left_camera.release()
        if right_camera is not None:
            right_camera.release()
        
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
    print("机器人超声波扫描系统 - 图像采集和位姿记录")
    print("="*50)
    print(f"[Config] 扫描位姿数: {NUM_POSES}")
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
        print("[Info] 按 Enter 键开始扫描，或按 Ctrl+C 退出...")
        input()
        
        # 执行扫描
        scan_with_positions()
        
        return True
    except KeyboardInterrupt:
        print("\n[Info] 用户中断")
        quit_flag = True
        return False
    except Exception as e:
        print(f"[Error] 程序出错: {e}")
        return False
    finally:
        cleanup()


if __name__ == "__main__":
    main()
