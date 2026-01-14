import numpy as np
import cv2
import time
import rtde_receive
from scipy.optimize import least_squares
from ur10e_kine import UR10eKine
from sys_path import *
from api.library_loader import *
from api.errors import ErrorCode, Error
from api.inu_sensor import InuSensor, ConnectionState
from api.image_frame import ImageFrame
from api.shared import StreamType

ROBOT_IP = '192.168.253.101'
BALL_RADIUS = 0.006  # 小球半径 (单位: 米) 
T_end2tcp = np.array([
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.07625],
    [1.0, 0.0, 0.0, 0.08900436787], 
    [0.0, 0.0, 0.0, 1.0]
])  # 末端到TCP的变换矩阵 (示例数据)
T_cam2tcp = np.array([
    [0.9992032522864902, 0.009194346124125006, -0.0388371551410461, -0.024890536721951274], 
    [0.03840857875850277, 0.48598030923302815, 0.8731253747976416, 0.028922078475610968], 
    [-0.010846275759530783, -0.8739213940836555, -0.4859462473000775, 0.20752911640646526],  
    [0.0, 0.0, 0.0, 1.0]
])

CAMERA_INTRINSICS = np.array([
    [545.046728971962693, 0.0, 534.957943925233735],
    [0.0, 510.000000000000000, 360.000000000000000],
    [0.0, 0.0, 1.0]
])

# 荧光小球颜色阈值 (HSV空间)
HSV_LOWER = np.array([0, 0, 25])   
HSV_UPPER = np.array([0, 0, 75]) 

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
            # 启动 Sensor
            self.sensor.start(None, dpe_params)
            
            # 创建 RGB 流
            self.rgb_stream = self.sensor.create_stream(StreamType.GENERAL_CAMERA)
            self.rgb_stream.init()
            self.rgb_stream.start()
            
            # 创建 Depth 流
            self.depth_stream = self.sensor.create_stream(StreamType.DEPTH)
            self.depth_stream.init()
            self.depth_stream.start()
            
            print("[Camera] Streams started successfully.")
            self.is_initialized = True
            return True
        except Exception as e:
            print(f"[Camera Error] Init failed: {e}")
            import traceback
            traceback.print_exc()
            return False

    def get_data(self):
        """
        获取最新的 RGB 和 Depth 数据 (带格式自适应)
        """
        if not self.is_initialized:
            return None, None

        try:
            rgb_frame = self.rgb_stream.frame
            depth_frame = self.depth_stream.frame

            if rgb_frame is None or depth_frame is None:
                return None, None

            # 打印调试信息 (方便排查分辨率问题)
            # print(f"RGB: {rgb_frame.width}x{rgb_frame.height} BPP={rgb_frame.bytes_per_pixel} | Depth: {depth_frame.width}x{depth_frame.height} BPP={depth_frame.bytes_per_pixel}")

            # --- 1. 处理 RGB (自适应通道数，防止 reshape 报错) ---
            if rgb_frame.buffer is None: return None, None
            
            channels = rgb_frame.bytes_per_pixel
            # 动态 reshape
            raw_rgb = np.array(rgb_frame.buffer, dtype=np.uint8).reshape(rgb_frame.height, rgb_frame.width, channels)
            
            if channels == 4:
                color_img = cv2.cvtColor(raw_rgb, cv2.COLOR_RGBA2BGR)
            elif channels == 3:
                color_img = cv2.cvtColor(raw_rgb, cv2.COLOR_RGB2BGR) 
            else:
                print(f"[Error] Unsupported RGB channels: {channels}")
                return None, None

            # --- 2. 处理 Depth (格式检查，防止读错数据) ---
            if depth_frame.buffer is None: return None, None
            
            # 只有当 BPP=2 (16bit) 时才认为它是距离数据
            if depth_frame.bytes_per_pixel == 2:
                raw_depth = np.frombuffer(depth_frame.buffer, dtype=np.uint16).reshape(depth_frame.height, depth_frame.width)
                # 转换为米 (float32)
                depth_img_meters = raw_depth.astype(np.float32) / 1000.0
            else:
                print(f"[Error] Depth format mismatch! Expected 2 bytes/pixel (uint16), got {depth_frame.bytes_per_pixel}")
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
        except:
            pass

def fit_sphere_from_depth(color_img, depth_img, K, ball_radius):
    if color_img is None or depth_img is None: return None

    # 1. 颜色分割
    hsv = cv2.cvtColor(color_img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)
    
    kernel = np.ones((3,3), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=2)
    
    # 2. 提取点云
    idxs = np.where(mask > 0)
    v_pixels = idxs[0] # y
    u_pixels = idxs[1] # x
    
    if len(v_pixels) < 50:
        print("[算法警告] 未检测到小球，请检查颜色阈值或光照。")
        return None
        
    points_3d = []
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    
    for i in range(len(v_pixels)):
        u, v = u_pixels[i], v_pixels[i]
        z = depth_img[v, u] 
        
        # 过滤无效深度 (例如太近<0.1m 或 太远>2.0m)
        if z <= 0.1 or z > 2.0: continue 
            
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        points_3d.append([x, y, z])
        
    points_3d = np.array(points_3d)
    if len(points_3d) < 10:
        print("[算法警告] 有效点云太少。")
        return None
        
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

def main():
    print("=== 初始化硬件 ===")
    
    try:
        kine_solver = UR10eKine(ROBOT_IP)
        rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
        print("[Robot] Connected.")
    except Exception as e:
        print(f"[Robot Error] Connection failed: {e}")
        return

    camera = InuCameraHandler()
    if not camera.initialize():
        return

    print("\n=== 开始验证 (请确保示教器TCP已设为[0,0,0,0,0,0]) ===")
    
    try:
        # --- 步骤 1: 物理真值 ---
        input("\n[1/2] 请移动机器人使探头【紧贴】小球。\n完成后按 Enter...")
        
        # 获取法兰位姿 (因为要求TCP设为0，所以这里读到的就是法兰位姿)
        pose_vec_touch = rtde_r.getActualTCPPose()
        print(f"接触位姿: {pose_vec_touch}")
        
        T_base_flange_touch = kine_solver.XYZRXRYRZ_to_Tmatrix(pose_vec_touch)
        offset_ball_center = np.array([0, 0, BALL_RADIUS, 1.0]) 
        
        # 核心公式: Base = T_base_flange * T_end2tcp * Offset
        P_GT_homo = T_base_flange_touch @ T_end2tcp @ offset_ball_center
        P_GT = np.array(P_GT_homo).flatten()[:3]
        
        print(f"基准坐标 (Base): {P_GT}")
        
        # --- 步骤 2: 视觉验证 ---
        while True:
            cmd = input("\n[2/2] 请移动到观测位置按 Enter (输入 'q' 退出): ")
            if cmd.lower() == 'q': break
                
            pose_vec_look = rtde_r.getActualTCPPose()
            T_base_flange_look = kine_solver.XYZRXRYRZ_to_Tmatrix(pose_vec_look)
            
            # 获取数据 (重试机制)
            color_img, depth_img = None, None
            for _ in range(20):
                color_img, depth_img = camera.get_data()
                if color_img is not None: break
                time.sleep(0.05)

            if color_img is None:
                print("[Error] 无法获取图像，请检查相机连接")
                continue

            # 算球心
            t0 = time.time()
            P_cam = fit_sphere_from_depth(color_img, depth_img, CAMERA_INTRINSICS, BALL_RADIUS)
            print(f"耗时: {time.time()-t0:.3f}s")
            
            if P_cam is None: continue
            
            print(f"视觉球心 (Cam): {P_cam}")
            
            # 核心公式: Base = T_base_flange * T_cam2tcp * P_cam
            # 使用你们规定的变量名 T_cam2tcp
            P_cam_homo = np.append(P_cam, 1.0)
            P_Vis_homo = T_base_flange_look @ T_cam2tcp @ P_cam_homo
            P_Vis = np.array(P_Vis_homo).flatten()[:3]
            
            # 结果
            error_vec = P_GT - P_Vis
            error_dist = np.linalg.norm(error_vec)
            
            print("-" * 30)
            print(f"真值: {P_GT}")
            print(f"测量: {P_Vis}")
            print(f"误差: {error_dist*1000:.2f} mm")
            print("-" * 30)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        camera.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()