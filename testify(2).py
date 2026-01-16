import cv2
import numpy as np
import csv
import time
import os
import sys
from camera.single_camera import SingleCamera
import rtde_control
import rtde_receive
from scipy.spatial.transform import Rotation as R

# ================= 1. 参数配置区域 =================
ROBOT_IP = "192.168.253.101"   
SAVE_DIR = "validation_data_inu" 
SENSOR_MASS = 0.820
SENSOR_COG = [0.007, -0.006, 0.07]
FREE_AXES = [1, 1, 1, 1, 1, 1]
FREE_FEATURE = [1, 1, 1, 0.5, 0.5, 0.5]

CHECKERBOARD_SIZE = (11, 8)  
SQUARE_SIZE = 0.015       

NORMALIZED_K = np.array([
    [545.046728971962693, 0,         534.957943925233735],
    [0,         510.000000000000000, 360.000000000000000],
    [0,         0,         1]
], dtype=np.float64)

dist_coeffs = np.array([0.00000, 0.00000, 0.00000, 0.00000, 0.00000], dtype=np.float64)

T_cam2tcp = np.array([
    [0.99920325, 0.00919435, -0.03883716, -0.02489054], 
    [0.03840858, 0.48598031, 0.87312537, 0.02892208], 
    [-0.01084628, -0.87392139, -0.48594625, 0.20752912],  
    [0.0, 0.0, 0.0, 1.0]
])

# ================= 2. 辅助函数 =================

def get_denormalized_matrix(norm_K, width, height):
    """如果内参是归一化的，将其转换为像素单位"""
    K = norm_K.copy()
    # 简单的判断：如果fx小于10，大概率是归一化的，需要乘分辨率
    if K[0, 0] < 10.0:
        print(f"[Info] 检测到归一化内参，正在根据分辨率 {width}x{height} 进行还原...")
        K[0, 0] *= width   # fx * w
        K[0, 2] *= width   # cx * w
        K[1, 1] *= height  # fy * h
        K[1, 2] *= height  # cy * h
    return K

def vector_to_matrix(pose_vec):
    t = np.array(pose_vec[:3])
    r_vec = np.array(pose_vec[3:])
    r_mat = R.from_rotvec(r_vec).as_matrix()
    T = np.eye(4)
    T[:3, :3] = r_mat
    T[:3, 3] = t
    return T

def run_verification_logic(image_files, robot_poses, final_K):
    if not image_files or len(image_files) != len(robot_poses):
        print("错误：没有数据或数据不匹配")
        return

    objp = np.zeros((CHECKERBOARD_SIZE[0] * CHECKERBOARD_SIZE[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD_SIZE[0], 0:CHECKERBOARD_SIZE[1]].T.reshape(-1, 2)
    objp = objp * SQUARE_SIZE

    computed_positions = []
    
    print(f"\n{'ID':<5} {'状态':<10} {'PnP平移(m)':<25}")
    print("-" * 50)

    for i, img_path in enumerate(image_files):
        img = cv2.imread(img_path)
        if img is None: continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD_SIZE, None)

        if ret:
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), 
                                        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
            
            retval, rvec, tvec = cv2.solvePnP(objp, corners2, final_K, dist_coeffs)

            # T_cam_to_board
            R_cam_board, _ = cv2.Rodrigues(rvec)
            T_cam_to_board = np.eye(4)
            T_cam_to_board[:3, :3] = R_cam_board
            T_cam_to_board[:3, 3] = tvec.flatten()

            # T_base_to_tcp
            T_base_to_tcp = vector_to_matrix(robot_poses[i])

            # 链式计算
            T_base_to_board = T_base_to_tcp @ T_cam2tcp @ T_cam_to_board
            
            computed_positions.append(T_base_to_board[:3, 3])
            print(f"{i:<5} {'OK':<10} {str(np.round(tvec.flatten(), 3)):<25}")
        else:
            print(f"{i:<5} {'Fail':<10} {'角点未检测到'}")

    if len(computed_positions) < 2:
        print("数据不足，无法计算误差。")
        return

    positions = np.array(computed_positions)
    mean_pos = np.mean(positions, axis=0)
    errors = np.linalg.norm(positions - mean_pos, axis=1)
    rmse = np.sqrt(np.mean(errors**2))

    print("\n" + "="*40)
    print(f" RMSE (均方根误差): {rmse*1000:.4f} mm")
    print("="*40 + "\n")

# ================= 3. 主程序 =================

def main():
    # --- 1. 连接机器人 ---
    print(f"正在连接机器人 {ROBOT_IP} ...")
    try:
        rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)
        rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
        rtde_c.setPayload(SENSOR_MASS, SENSOR_COG)
        print("机器人连接成功！")
    except Exception as e:
        print(f"机器人连接失败: {e}")
        return

    # --- 2. 连接 InuSensor 相机 ---
    print("正在初始化 InuSensor 相机...")
    # 注意：clear_on_init=False 防止误删之前的数据，如果需要每次清空请设为 True
    camera = SingleCamera(imgs_path=SAVE_DIR + "/", clear_on_init=False)
    
    if not camera.initialize():
        print("相机初始化失败，请检查连接。")
        return

    # --- 3. 准备工作 ---
    if not os.path.exists(SAVE_DIR):
        os.makedirs(SAVE_DIR)
    
    current_session_images = []
    current_session_poses = []
    
    is_dragging = False
    real_K = None # 将在获取第一帧后计算
    
    print("\n" + "="*50)
    print(" [D] - 切换 '拖动模式' (Freedrive)")
    print(" [S] - '停止拖动' 并 '保存数据'")
    print(" [V] - '验证误差' (RMSE)")
    print(" [Q] - 退出")
    print("="*50)

    try:
        while True:
            # --- 获取图像 (替换了 cv2 read) ---
            raw_frame = camera.get_frame()
            if raw_frame is None:
                time.sleep(0.01)
                continue
            
            # 转换为 OpenCV 格式 (BGR)
            frame = camera._frame_to_image(raw_frame)
            if frame is None:
                continue

            # --- 首次运行时计算真实的内参矩阵 ---
            if real_K is None:
                h, w = frame.shape[:2]
                print(f"\n[Info] 相机分辨率: {w}x{h}")
                real_K = get_denormalized_matrix(NORMALIZED_K, w, h)
                print(f"[Info] 使用的内参矩阵 (Pixels):\n{real_K}\n")

            # --- UI 显示 ---
            display_frame = frame.copy()
            
            # 画角点预览
            gray = cv2.cvtColor(display_frame, cv2.COLOR_BGR2GRAY)
            ret_corn, corners = cv2.findChessboardCorners(gray, CHECKERBOARD_SIZE, None)
            if ret_corn:
                cv2.drawChessboardCorners(display_frame, CHECKERBOARD_SIZE, corners, ret_corn)

            status_text = "MODE: DRAGGING" if is_dragging else "MODE: LOCKED"
            color = (0, 255, 0) if is_dragging else (0, 0, 255)
            cv2.putText(display_frame, status_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            cv2.putText(display_frame, f"Count: {len(current_session_images)}", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

            cv2.imshow('InuSensor Calibration', display_frame)
            key = cv2.waitKey(1) & 0xFF

            # --- 逻辑控制 ---

            # 开启拖动
            if key == ord('d'):
                if not is_dragging:
                    rtde_c.freedriveMode(FREE_AXES, FREE_FEATURE)
                    is_dragging = True
                    print("--> 进入拖动模式")

            # 保存数据
            elif key == ord('s'):
                if is_dragging:
                    rtde_c.endFreedriveMode()
                    is_dragging = False
                    time.sleep(0.5) # 等待稳定
                
                # 获取机器人位姿 (基座->法兰)
                # 务必确保示教器 TCP 为 [0,0,0,0,0,0]
                tcp_pose = rtde_r.getActualTCPPose()
                
                # 保存图像 (直接用当前帧)
                filename = f"val_{int(time.time())}.jpg"
                filepath = os.path.join(SAVE_DIR, filename)
                cv2.imwrite(filepath, frame) # 保存原始清晰图像
                
                current_session_images.append(filepath)
                current_session_poses.append(tcp_pose)
                
                print(f"已保存: {filename}")

            # 验证
            elif key == ord('v'):
                if len(current_session_images) > 0:
                    run_verification_logic(current_session_images, current_session_poses, real_K)
                else:
                    print("无数据，请先采集。")

            # 退出
            elif key == ord('q'):
                break

    finally:
        # 清理资源
        if is_dragging:
            rtde_c.endFreedriveMode()
        
        rtde_c.stopScript()
        rtde_c.disconnect()
        rtde_r.disconnect()
        
        # 关闭 InuSensor
        camera.close()
        cv2.destroyAllWindows()
        print("程序已安全退出。")

if __name__ == "__main__":
    main()