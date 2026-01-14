import open3d as o3d
import numpy as np
import copy
from kinematics.ur10e_kine import UR10eKine
import rtde_control
import rtde_receive

def create_coordinate_frame(T, size=0.1, name="frame"):
    """
    创建一个带有颜色标记的坐标系
    红色=X, 绿色=Y, 蓝色=Z
    """
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size, origin=[0, 0, 0])
    frame.transform(T)
    return frame

def create_connection_line(start_point, end_point, color=[0.5, 0.5, 0.5]):
    """
    在两个坐标系原点之间画一条连线，表示刚性连接关系
    """
    lines = [[0, 1]]
    points = [start_point, end_point]
    colors = [color]
    
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)
    return line_set

def get_trajectory_pose(trajectory_data, count):
    """根据计数索引获取轨迹矩阵"""

    if 0 <= count < 104:
        flat_pose = trajectory_data[count]
        return flat_pose.reshape(4, 4)
    else:
        print(f"[ERROR] 索引 {count} 溢出")
        return np.eye(4)

def visualize_robot_system(T_base_end, T_end_tool, T_end_cam, pcd_in_cam_frame=None, pcd_global_1=None):
    """
    可视化机器人系统的各个坐标系和点云
    
    参数:
    T_base_end: 4x4 numpy array, 机器人基座到末端的变换矩阵 (正运动学结果)
    T_end_tool: 4x4 numpy array, 末端到工具尖端的变换 (TCP标定值)
    T_end_cam:  4x4 numpy array, 末端到相机的变换 (手眼标定外参)
    pcd_in_cam_frame: open3d.geometry.PointCloud, 相机坐标系下的原始点云
    """
    T_best_transform = np.loadtxt("/home/zzz/ros2_ws/src/Camera_example/CPP/app/PCLProject/debug_pcd/best_transform.txt").reshape(4,4)
    trajectory_data = np.loadtxt("point_cloud/path_matrices.txt")
    geometries = []

    # 1. 计算全局位姿 (都在 Base 坐标系下描述)
    # T_base (Identity)
    T_base = np.eye(4)
    
    # T_tool_global = T_base_end * T_end_tool
    T_tool_global = T_base_end @ T_end_tool
    
    # T_cam_global = T_base_end * T_end_cam
    T_cam_global = T_base_end @ T_end_cam
    print(f"T_cam_global: {T_cam_global}")

    T_best_transform_inv = np.linalg.inv(T_best_transform)
    for i in range(1):
        print(f"======================== {i}")
        T_temp_i = get_trajectory_pose(trajectory_data, i)
        T_temp_j = T_best_transform_inv @ T_temp_i
        T_base_traj = T_cam_global @ T_temp_j 
        # [ 0.83011146  0.30991028  0.46353975  0.38348728]
        # [ 0.45088657 -0.86216298 -0.23103316 -0.6913217 ]
        # [ 0.32804712  0.40078754 -0.85542506  0.223666  ]
        # [ 0.          0.          0.          1.        ]

        # print(f"T_base_traj: {T_base_traj}")
        # t_NEW_1 = T_base_traj
        # t_NEW_1[:3, :3] = np.eye(3)

        T_tcp2end = np.linalg.inv(T_end_tool)
        T_tcp2base_target = T_base_traj @ T_tcp2end
        print(f"T_tcp2base_target: {T_tcp2base_target}")
        geometries.append(create_coordinate_frame(T_base_traj, size=0.05))
        geometries.append(create_coordinate_frame(T_tcp2base_target, size=0.05))
        # geometries.append(create_coordinate_frame(t_NEW_1, size=0.1))
    # 2. 生成坐标系可视化
    # 基座 (画大一点，0.2m)
    geometries.append(create_coordinate_frame(T_base, size=0.2))
    
    # 末端 (0.1m)
    geometries.append(create_coordinate_frame(T_base_end, size=0.1))
    
    # 工具 (0.05m，小一点)
    geometries.append(create_coordinate_frame(T_tool_global, size=0.08))
    
    # 相机 (0.05m)
    geometries.append(create_coordinate_frame(T_cam_global, size=0.08))

    # 3. 画连线 (表示层级关系)
    # Base -> End
    geometries.append(create_connection_line(T_base[:3, 3], T_base_end[:3, 3], color=[0.8, 0.8, 0.8]))
    # End -> Tool (黄色线)
    geometries.append(create_connection_line(T_base_end[:3, 3], T_tool_global[:3, 3], color=[1, 1, 0]))
    # End -> Camera (青色线)
    geometries.append(create_connection_line(T_base_end[:3, 3], T_cam_global[:3, 3], color=[0, 1, 1]))

    # 4. 处理点云
    if pcd_in_cam_frame is not None:
        # 深拷贝一份，以免修改原始数据
        pcd_global = copy.deepcopy(pcd_in_cam_frame)
        pcd_global_1 = copy.deepcopy(pcd_global_1)
        # 核心：将点云从 相机坐标系 变换到 基座坐标系

        # T_temp = np.array([
        #  [-1, 0, 0, 0],
        #  [0, 1, 0, 0],
        #  [0, 0, 1, 0],
        #  [0.0, 0.0, 0.0, 1.0]
        # ])
        # pcd_global.transform(T_temp)

        pcd_global.transform(T_cam_global)
        
        pcd_global_1.transform(T_best_transform_inv)
        pcd_global_1.transform(T_cam_global)

        # 给点云涂个色区分一下 (比如淡蓝色)
        # pcd_global.paint_uniform_color([0.6, 0.8, 1.0])
        geometries.append(pcd_global)
        geometries.append(pcd_global_1)

    # 5. 启动可视化
    print("="*50)
    print("可视化说明:")
    print("  原点(大) : 机器人基座 (Base)")
    print("  坐标系(中) : 机器人末端法兰 (End-Effector)")
    print("  坐标系(小) : 工具尖端 (Tool) [黄线连接]")
    print("  坐标系(小) : 相机光心 (Camera) [青线连接]")
    print("  蓝色点云   : 变换后的局部扫描数据")
    print("="*50)
    
    o3d.visualization.draw_geometries(geometries, window_name="Robot System Verification", width=1024, height=768)

# ==========================================
# 示例数据 (请用你的真实数据替换这里)
# ==========================================
if __name__ == "__main__":
    # 1. 模拟数据：假设机器人伸向前方
    # T_base_end: 末端相对于基座的坐标变换矩阵
    robot_ip = '192.168.253.101'
    ur10e_kine = UR10eKine(robot_ip)
    rtde_c = ur10e_kine.rtde_c
    rtde_r = ur10e_kine.rtde_r
    q = rtde_r.getActualQ()
    T_base_end = ur10e_kine.FK_wo_end(q)

    # T_base_end = np.array([
    #     [ 0.53090236,  0.53425639,  0.65780909 , 0.26014832],
    #     [-0.84741731,  0.32997301,  0.41593476, -0.78573554],
    #     [ 0.00515656, -0.77825955 , 0.62792156 , 0.22201456],
    #     [ 0.         , 0.      ,    0.   ,      1.        ]
    # ])

    # 2. 标定数据 (手眼标定结果)
    # 假设rgb相机相对于法兰
    T_end_rgbcam = np.array([
         [0.9992032522864902, 0.009194346124125006, -0.0388371551410461, -0.024890536721951274],
         [0.03840857875850277, -0.48598030923302815, 0.8731253747976416, 0.028922078475610968],
         [-0.010846275759530783, -0.8739213940836555, -0.4859462473000775, 0.20752911640646526],
         [0.0, 0.0, 0.0, 1.0]
    ])                                                                                                                                                                                                                                                                                              

    #假设深度相机相对于rgb相机
    T_rgbcam_depthcam = np.array([
         [1, 0, 0, 0.05024764065403759],
         [0, 1, 0, -0.0002759525896356895],
         [0, 0, 1, 0.002105396482748959],
         [0.0, 0.0, 0.0, 1.0]
    ])

    T_end_cam = T_end_rgbcam @ T_rgbcam_depthcam
    print(f"T_end_cam {T_end_cam}")
    # 3. 工具数据 (TCP)
    # 假设工具在法兰中心延伸 20cm
    T_end_tool = np.array([
         [0, 1, 0, 0],
         [0, 0, 1, 0.07625],
         [1, 0, 0, 0.08900436787],
         [0.0, 0.0, 0.0, 1.0]
    ])
    #T_end_tool[:3, 3] = [0.0, 0.0, 0.2] 
    
    # 4. 加载你的点云
    # 这里我们创建一个模拟点云 (相机前方 0.5m 处的一个球
    # 实际使用时: 
    pcd = o3d.io.read_point_cloud("/home/zzz/ros2_ws/src/Camera_example/CPP/app/PCLProject/debug_pcd/source_10.pcd")
    #pcd_local = o3d.geometry.TriangleMesh.create_sphere(radius=0.05).sample_points_uniformly(1000)
    # 在相机坐标系下，物体在 Z 轴正方向 0.3m 处 (假设相机Z轴朝前)
    # pcd_local.translate([0, 0, 0.3]) 
    pcd_glabal = o3d.io.read_point_cloud("/home/zzz/ros2_ws/src/Camera_example/CPP/app/PCLProject/pcd_data/new_target.pcd")
    # --- 运行可视化 ---
    visualize_robot_system(T_base_end, T_end_tool, T_end_cam, pcd, pcd_glabal)