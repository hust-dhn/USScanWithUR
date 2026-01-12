import sys
import os
import numpy as np
import cv2
import time
import threading
from scipy.optimize import least_squares
import rtde_receive
import rtde_control  # 引入控制接口用于拖动
from ur10e_kine import UR10eKine
from sys_path import *
from api.library_loader import *
from api.errors import ErrorCode, Error
from api.inu_sensor import InuSensor, ConnectionState
from api.image_frame import ImageFrame
from api.shared import StreamType

# ================= 配置区域 =================

ROBOT_IP = '192.168.253.101'
BALL_RADIUS = 0.006  # 小球半径 (单位: 米)

# 默认 HSV 阈值 (可以通过调试模式更新这里)
HSV_LOWER = np.array([0, 0, 25])   
HSV_UPPER = np.array([0, 0, 75]) 

# 拖动模式参数
FREE_AXES = [1, 1, 1, 1, 1, 1] # 允许所有轴移动
FREE_FEATURE = [1, 1, 1, 0.5, 0.5, 0.5] # 基座坐标系

# 变换矩阵
T_end2tcp = np.array([
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.07625],
    [1.0, 0.0, 0.0, 0.08900436787], 
    [0.0, 0.0, 0.0, 1.0]
])

T_cam2tcp = np.array([
    [0.99920325, 0.00919435, -0.03883716, -0.02489054], 
    [0.03840858, 0.48598031, 0.87312537, 0.02892208], 
    [-0.01084628, -0.87392139, -0.48594625, 0.20752912],  
    [0.0, 0.0, 0.0, 1.0]
])

CAMERA_INTRINSICS = np.array([
    [545.04672897, 0.0, 534.95794393],
    [0.0, 510.00000000, 360.00000000],
    [0.0, 0.0, 1.0]
])

# ================= 辅助类与函数 =================

class RobotHandler:
    """处理机器人连接和自由拖动"""
    def __init__(self, ip):
        print(f"[Robot] Connecting to {ip}...")
        self.rtde_c = rtde_control.RTDEControlInterface(ip)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(ip)
        self.kine = UR10eKine(ip)
        self.is_freedrive = False
        print("[Robot] Connected.")

    def toggle_freedrive(self):
        if not self.is_freedrive:
            self.rtde_c.freedriveMode(FREE_AXES, FREE_FEATURE)
            self.is_freedrive = True
            print("\n>>> [模式] 自由拖动已开启 (Free Drive ON)")
        else:
            self.rtde_c.endFreedriveMode()
            self.is_freedrive = False
            print("\n>>> [模式] 自由拖动已锁定 (Free Drive OFF)")
    
    def stop(self):
        if self.is_freedrive:
            self.rtde_c.endFreedriveMode()
        self.rtde_c.stopScript()

class InuCameraHandler:
    def __init__(self):
        self.sensor = None
        self.rgb_stream = None
        self.depth_stream = None
        self.is_initialized = False

    def initialize(self):
        print("[Camera] Initializing InuSensor...")
        try:
            self.sensor = InuSensor()
            hw_info, dpe_params = self.sensor.init()
            self.sensor.start(None, dpe_params)
            
            self.rgb_stream = self.sensor.create_stream(StreamType.GENERAL_CAMERA)
            self.rgb_stream.init()
            self.rgb_stream.start()
            
            self.depth_stream = self.sensor.create_stream(StreamType.DEPTH)
            self.depth_stream.init()
            self.depth_stream.start()
            
            print("[Camera] Streams started.")
            self.is_initialized = True
            return True
        except Exception as e:
            print(f"[Camera Error] Init failed: {e}")
            return False

    def get_data(self):
        if not self.is_initialized: return None, None
        try:
            rgb_frame = self.rgb_stream.frame
            depth_frame = self.depth_stream.frame

            if rgb_frame is None or depth_frame is None: return None, None
            if rgb_frame.buffer is None or depth_frame.buffer is None: return None, None

            # --- 1. RGB 处理 ---
            # 形状通常是 (Height, Width, Channels)
            h, w = rgb_frame.height, rgb_frame.width
            channels = rgb_frame.bytes_per_pixel
            raw_rgb = np.array(rgb_frame.buffer, dtype=np.uint8).reshape(h, w, channels)
            
            if channels == 4:
                color_img = cv2.cvtColor(raw_rgb, cv2.COLOR_RGBA2BGR)
            elif channels == 3:
                color_img = cv2.cvtColor(raw_rgb, cv2.COLOR_RGB2BGR)
            else:
                return None, None

            # --- 2. Depth 处理 (修正Width/Height和Reshape) ---
            # 这里的逻辑修正了您遇到的问题：
            # 必须用 uint16 读取，才能得到正确的深度值，此时 reshape 不需要末尾的 '2'
            if depth_frame.bytes_per_pixel == 2:
                # 形状: (Height, Width)
                raw_depth = np.frombuffer(depth_frame.buffer, dtype=np.uint16).reshape(depth_frame.height, depth_frame.width)
                depth_img_meters = raw_depth.astype(np.float32) / 1000.0
            else:
                print(f"[Err] Depth BPP is {depth_frame.bytes_per_pixel}, expected 2 (uint16)")
                return None, None

            return color_img, depth_img_meters

        except Exception as e:
            print(f"[Camera Error] Get data failed: {e}")
            return None, None

    def close(self):
        try:
            if self.rgb_stream: self.rgb_stream.stop(); self.rgb_stream.terminate()
            if self.depth_stream: self.depth_stream.stop(); self.depth_stream.terminate()
            if self.sensor: self.sensor.stop(); self.sensor.terminate()
        except: pass

def tune_hsv_interactive(camera):
    """
    交互式 HSV 阈值调试工具
    """
    print("\n" + "="*50)
    print("   进入 HSV 调试模式")
    print("   1. 画面会实时刷新")
    print("   2. 将小球放入视野")
    print("   3. 按 's' 键冻结画面并框选小球")
    print("   4. 按 'q' 键退出调试模式")
    print("="*50 + "\n")

    cv2.namedWindow("HSV Tuner", cv2.WINDOW_NORMAL)
    
    while True:
        color_img, _ = camera.get_data()
        if color_img is None:
            time.sleep(0.01)
            continue
            
        display_img = color_img.copy()
        cv2.putText(display_img, "Press 's' to select ROI, 'q' to quit", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        cv2.imshow("HSV Tuner", display_img)
        key = cv2.waitKey(10) & 0xFF
        
        if key == ord('q'):
            break
        elif key == ord('s'):
            print("\n>>> 请在弹出的窗口中用鼠标框选小球，然后按 Enter 或 Space 确认...")
            roi = cv2.selectROI("HSV Tuner", color_img, showCrosshair=True, fromCenter=False)
            x, y, w, h = roi
            
            if w > 0 and h > 0:
                roi_img = color_img[y:y+h, x:x+w]
                hsv_roi = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV)
                
                # 计算 Min/Max
                h_min, s_min, v_min = np.min(hsv_roi[:,:,0]), np.min(hsv_roi[:,:,1]), np.min(hsv_roi[:,:,2])
                h_max, s_max, v_max = np.max(hsv_roi[:,:,0]), np.max(hsv_roi[:,:,1]), np.max(hsv_roi[:,:,2])
                
                # 适当放宽范围 (+/- 15)
                lower = np.array([max(0, h_min-10), max(0, s_min-20), max(0, v_min-20)])
                upper = np.array([min(180, h_max+10), min(255, s_max+20), min(255, v_max+20)])
                
                print("\n" + "*"*40)
                print("计算出的建议阈值 (请复制替换代码顶部的常量):")
                print(f"HSV_LOWER = np.array([{lower[0]}, {lower[1]}, {lower[2]}])")
                print(f"HSV_UPPER = np.array([{upper[0]}, {upper[1]}, {upper[2]}])")
                print("*"*40 + "\n")
                
                # 临时应用以便测试
                global HSV_LOWER, HSV_UPPER
                HSV_LOWER, HSV_UPPER = lower, upper
                print(">>> 已临时应用新阈值，按任意键继续实时预览...")
                cv2.waitKey(0)
            else:
                print("未选择有效区域")

    cv2.destroyWindow("HSV Tuner")

def fit_sphere_from_depth(color_img, depth_img, K, ball_radius):
    if color_img is None or depth_img is None: return None

    # 1. 颜色分割
    hsv = cv2.cvtColor(color_img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)
    
    # 形态学去噪
    kernel = np.ones((3,3), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=2)
    
    # 可视化掩膜 (调试用)
    cv2.imshow("Mask Debug", mask)
    cv2.waitKey(1)
    
    # 2. 提取点云
    idxs = np.where(mask > 0)
    v_pixels = idxs[0] # y
    u_pixels = idxs[1] # x
    
    if len(v_pixels) < 30:
        # print("[算法] 未检测到小球")
        return None
        
    points_3d = []
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    
    # 为了速度，可以进行降采样，比如每隔3个点取一个
    step = 2 
    for i in range(0, len(v_pixels), step):
        u, v = u_pixels[i], v_pixels[i]
        z = depth_img[v, u] 
        
        # 过滤无效深度
        if z <= 0.05 or z > 1.5: continue 
            
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        points_3d.append([x, y, z])
        
    points_3d = np.array(points_3d)
    if len(points_3d) < 10: return None
        
    # 3. 拟合
    def residuals(center, points, radius):
        return np.linalg.norm(points - center, axis=1) - radius
        
    initial_guess = np.mean(points_3d, axis=0)
    initial_guess[2] += ball_radius 
    
    try:
        res = least_squares(residuals, initial_guess, args=(points_3d, ball_radius))
        if not res.success: return None
        return res.x
    except:
        return None

# ================= 主流程 =================

def main():
    print("=== 系统初始化 ===")
    
    # 1. 初始化机器人和相机
    try:
        robot = RobotHandler(ROBOT_IP)
        camera = InuCameraHandler()
        if not camera.initialize():
            print("相机初始化失败")
            return
    except Exception as e:
        print(f"初始化异常: {e}")
        return

    # 2. 询问是否进行HSV调试
    print("\n是否需要调试 HSV 阈值？(y/n)")
    choice = input("请输入: ").strip().lower()
    if choice == 'y':
        tune_hsv_interactive(camera)
    
    print("\n=== 开始验证流程 ===")
    print("操作说明: 在步骤中，您可以随时按 'd' 键切换【自由驱动】模式来拖动机械臂。")
    print("注意: 采集数据前，请务必再次按 'd' 锁住机械臂，确保稳定！")

    try:
        # --- 步骤 1: 物理真值 ---
        while True:
            cmd = input("\n[1/2] 移动探头紧贴小球。\n(按 'd' 切换拖动, 准备好后按 Enter 记录): ").strip().lower()
            if cmd == 'd':
                robot.toggle_freedrive()
            elif cmd == '':
                if robot.is_freedrive:
                    print("请先按 'd' 关闭自由驱动模式！")
                    continue
                break
        
        # 记录真值
        pose_vec_touch = robot.rtde_r.getActualTCPPose()
        print(f"接触位姿: {pose_vec_touch}")
        
        T_base_flange_touch = robot.kine.XYZRXRYRZ_to_Tmatrix(pose_vec_touch)
        offset_ball_center = np.array([0, 0, BALL_RADIUS, 1.0]) 
        
        P_GT_homo = T_base_flange_touch @ T_end2tcp @ offset_ball_center
        P_GT = np.array(P_GT_homo).flatten()[:3]
        print(f"基准坐标 (Base): {P_GT}")
        
        # --- 步骤 2: 视觉验证 ---
        cv2.namedWindow("Mask Debug", cv2.WINDOW_NORMAL)
        
        while True:
            cmd = input("\n[2/2] 移动到观测位置。\n(按 'd' 切换拖动, 按 Enter 拍摄, 输入 'q' 退出): ").strip().lower()
            
            if cmd == 'q':
                break
            elif cmd == 'd':
                robot.toggle_freedrive()
                continue
            
            if robot.is_freedrive:
                print("请先按 'd' 关闭自由驱动模式！")
                continue

            # 获取位姿
            pose_vec_look = robot.rtde_r.getActualTCPPose()
            T_base_flange_look = robot.kine.XYZRXRYRZ_to_Tmatrix(pose_vec_look)
            
            # 获取图像 (尝试多次)
            color_img, depth_img = None, None
            for _ in range(10):
                color_img, depth_img = camera.get_data()
                if color_img is not None: break
                time.sleep(0.05)

            if color_img is None:
                print("无法获取图像")
                continue

            # 算球心
            t0 = time.time()
            P_cam = fit_sphere_from_depth(color_img, depth_img, CAMERA_INTRINSICS, BALL_RADIUS)
            
            if P_cam is None: 
                print(">>> 未检测到小球 (请检查 Mask Debug 窗口)")
                continue
            
            print(f"视觉球心 (Cam): {P_cam} | 耗时: {time.time()-t0:.3f}s")
            
            # 转换到基座
            P_cam_homo = np.append(P_cam, 1.0)
            P_Vis_homo = T_base_flange_look @ T_cam2tcp @ P_cam_homo
            P_Vis = np.array(P_Vis_homo).flatten()[:3]
            
            # 结果对比
            error_vec = P_GT - P_Vis
            error_dist = np.linalg.norm(error_vec)
            
            print("-" * 30)
            print(f"真值: {P_GT}")
            print(f"测量: {P_Vis}")
            print(f"误差: {error_dist*1000:.2f} mm")
            print("-" * 30)

    except KeyboardInterrupt:
        print("用户中断")
    finally:
        robot.stop()
        camera.close()
        cv2.destroyAllWindows()
        print("程序结束")

if __name__ == "__main__":
    main()