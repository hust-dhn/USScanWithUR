import numpy as np
import cv2
import time
import rtde_receive
from scipy.optimize import least_squares
from ur10e_kine import UR10eKine

ROBOT_IP = '192.168.253.101'
BALL_RADIUS = 0.006  # 小球半径 (单位: 米) -> 请务必用游标卡尺测量准确
T_end2tcp = np.array([
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.07625],
    [1.0, 0.0, 0.0, 0.08900436787], 
    [0.0, 0.0, 0.0, 1.0]
])  # 末端到TCP的变换矩阵 (示例数据)
T_cam2tcp = np.array([
    [0.9992032522864902, 0.009194346124125006, -0.0388371551410461, -0.024890536721951274],  # 示例数据
    [0.03840857875850277, 0.48598030923302815, 0.8731253747976416, 0.028922078475610968], 
    [-0.010846275759530783, -0.8739213940836555, -0.4859462473000775, 0.20752911640646526],  
    [0.0, 0.0, 0.0, 1.0]
])

CAMERA_INTRINSICS = np.array([
    [545.046728971962693, 0.0, 534.957943925233735],
    [0.0, 510.000000000000000, 360.000000000000000],
    [0.0, 0.0, 1.0]
])

# 荧光小球颜色阈值 (HSV空间) -> 需要根据实际光照调整
HSV_LOWER = np.array([0, 0, 25])   # 示例: 黄绿色下限
HSV_UPPER = np.array([0, 0, 75]) # 示例: 黄绿色上限

def get_camera_data():
    """
    模拟获取相机数据的接口。
    实际使用时，请替换为你的 RealSense 或其他相机的 SDK 调用代码。
    """
    print(
"[系统提示]正在捕获图像 (此处需接入真实相机SDK)..."
)
    
    # --- 占位代码 (请替换) ---
    # cap = cv2.VideoCapture(0) ...
    # ret, color_frame = cap.read() ...
    # depth_frame = ... (获取深度图, 单位需转换为米)
    
    # 这里仅生成测试数据用于代码跑通
    # 实际部署请删除以下模拟代码
    color_frame = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.circle(color_frame, (320, 240), 30, (0, 255, 255), -1) # 画个假球
    depth_frame = np.ones((480, 640), dtype=np.float32) * 0.5 # 假设深度0.5米
    return color_frame, depth_frame

def fit_sphere_from_depth(color_img, depth_img, K, ball_radius):
    # 1. 颜色分割
    hsv = cv2.cvtColor(color_img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)
    # 形态学去噪
    kernel = np.ones((3,3), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=2)
    # 2. 提取点云
    idxs = np.where(mask > 0)
    v_pixels = idxs[0] # y
    u_pixels = idxs[1] # x
    if len(v_pixels) < 50:
        print("[警告] 未检测到足够的小球像素，请检查HSV阈值或光照。")
        return None
    points_3d = []
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    for i in range(len(v_pixels)):
        u, v = u_pixels[i], v_pixels[i]
        z = depth_img[v, u]
        # 过滤无效深度
        if z <= 0.1 or z > 3.0: 
            continue 
        # 反投影: Pixel -> Camera
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        points_3d.append([x, y, z])
    points_3d = np.array(points_3d)
    if len(points_3d) < 10:
        print("[警告] 有效深度点太少，无法拟合。")
        return None
    # 3. 最小二乘拟合球心 (约束半径)
    # 残差函数: dist(P, center) - R
    def residuals(center, points, radius):
        dist = np.linalg.norm(points - center, axis=1)
        return dist - radius
    # 初始猜测: 点云均值 + 半径补偿
    initial_guess = np.mean(points_3d, axis=0)
    initial_guess[2] += ball_radius 
    # 优化
    res = least_squares(residuals, initial_guess, args=(points_3d, ball_radius))
    if not res.success:
        print("[警告] 拟合失败。")
        return None
       
    return res.x # 返回拟合后的球心 [x, y, z]

def main():

    print("正在连接机器人...")
    try:
        kine_solver = UR10eKine(ROBOT_IP)
        # 额外实例化一个接收接口用于获取实时位姿 (因为 UR10eKine 没直接暴露 getActualTCPPose)
        rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
    except Exception as e:
        print(f"连接机器人失败: {e}")
        return

    print("\n=== 开始手眼标定验证程序 (荧光小球方案) ===")
    # --- 步骤 1: 获取物理真值 (Ground Truth) ---
    input("\n[步骤 1/2] 请手动移动机器人，使探头圆面中心【紧贴】小球表面。\n完成后按 Enter 键记录真值...")
    # 读取法兰位姿 (Vector: x,y,z,rx,ry,rz)
    pose_vec_touch = rtde_r.getActualTCPPose()
    print(f"接触位姿读取成功: {pose_vec_touch}")
    # 利用你的类将向量转为矩阵
    T_base_flange_touch = kine_solver.XYZRXRYRZ_to_Tmatrix(pose_vec_touch)
    # 计算球心物理坐标 P_GT
    # 逻辑: 基座 -> 法兰 -> 探头 -> 沿Z轴前伸半径距离(球心)
    offset_ball_center = np.array([0, 0, BALL_RADIUS, 1.0]) # 假设探头Z轴正对球心
    P_GT_homo = T_base_flange_touch @ T_end2tcp @ offset_ball_center
    P_GT = np.array(P_GT_homo).flatten()[:3]
    print(f"【基准】计算得到的球心物理坐标 (Base系):")
    print(f"X: {P_GT[0]:.4f}, Y: {P_GT[1]:.4f}, Z: {P_GT[2]:.4f}")
    
    # --- 步骤 2: 视觉验证 (Visual Verification) ---
    while True:
        cmd = input("\n[步骤 2/2] 请将机器人移动到相机能看到小球的位置。\n按 Enter 拍摄并验证 (输入 'q' 退出): ")
        if cmd.lower() == 'q':
            break
            
        # 1. 读取当前拍照位置的法兰位姿
        pose_vec_look = rtde_r.getActualTCPPose()
        T_base_flange_look = kine_solver.XYZRXRYRZ_to_Tmatrix(pose_vec_look)
        
        # 2. 获取图像 (RGB + Depth)
        color_img, depth_img = get_camera_data()
        
        # 3. 算法计算球心 (相机系)
        t0 = time.time()
        P_cam = fit_sphere_from_depth(color_img, depth_img, CAMERA_INTRINSICS, BALL_RADIUS)
        print(f"算法耗时: {time.time() - t0:.3f}秒")
        
        if P_cam is None:
            print("视觉识别失败，请调整位置重试。")
            continue
            
        print(f"视觉球心 (Cam系): {P_cam}")
        
        # 4. 坐标链转换: Cam -> Flange -> Base
        # P_Vis = T_Base_Flange * T_Flange_Cam * P_Cam
        P_cam_homo = np.append(P_cam, 1.0)
        P_Vis_homo = T_base_flange_look @ T_cam2tcp @ P_cam_homo
        P_Vis = np.array(P_Vis_homo).flatten()[:3]
        
        # 5. 误差对比
        error_vec = P_GT - P_Vis
        error_dist = np.linalg.norm(error_vec)
        
        print("-" * 40)
        print(f"【对比结果】")
        print(f"物理真值 (Base): {P_GT}")
        print(f"视觉计算 (Base): {P_Vis}")
        print(f"误差向量 (XYZ):  [{error_vec[0]*1000:.2f}, {error_vec[1]*1000:.2f}, {error_vec[2]*1000:.2f}] mm")
        print(f"综合距离误差:    {error_dist * 1000:.2f} mm")
        print("-" * 40)
        
        if error_dist < 0.005:
            print(">>> 验证通过！精度极佳 (<5mm) <<<")
        elif error_dist < 0.010:
            print(">>> 验证尚可，精度一般 (<10mm) <<<")
        else:
            print(">>> 偏差较大，建议检查标定或探头偏移 <<<")

if __name__ == "__main__":
    main()