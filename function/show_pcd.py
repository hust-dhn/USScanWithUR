import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

def diagnose_point_cloud(filename):
    print(f"诊断点云文件: {filename}")
    
    # 1. 加载点云
    try:
        pcd = o3d.io.read_point_cloud(filename)
    except Exception as e:
        print(f"加载失败: {e}")
        return
    
    # 2. 基本信息
    points = np.asarray(pcd.points)
    print(f"\n=== 基本信息 ===")
    print(f"点数: {len(points)}")
    print(f"是否有颜色: {pcd.has_colors()}")
    print(f"是否有法线: {pcd.has_normals()}")
    
    if len(points) == 0:
        print("错误: 点云为空!")
        return
    
    # 3. 坐标统计
    print(f"\n=== 坐标统计 ===")
    for i, axis in enumerate(['X', 'Y', 'Z']):
        data = points[:, i]
        print(f"{axis}: min={data.min():.3f}, max={data.max():.3f}, "
              f"mean={data.mean():.3f}, std={data.std():.3f}")
    
    # 4. 可视化数据分布
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    
    # XYZ直方图
    for i, axis in enumerate(['X', 'Y', 'Z']):
        ax = axes[i//2, i%2]
        data = points[:, i]
        ax.hist(data, bins=50, alpha=0.7)
        ax.set_title(f'{axis}轴分布')
        ax.set_xlabel(f'{axis}值')
        ax.set_ylabel('频次')
    
    # 散点图（前两个维度）
    ax = axes[1, 1]
    ax.scatter(points[:, 0], points[:, 1], s=1, alpha=0.5)
    ax.set_title('XY平面投影')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.axis('equal')
    
    plt.tight_layout()
    plt.show()
    
    # 5. 检查可能的坐标系问题
    print(f"\n=== 坐标系分析 ===")
    
    # 计算主方向
    centroid = points.mean(axis=0)
    centered = points - centroid
    cov = np.cov(centered.T)
    eigenvalues, eigenvectors = np.linalg.eig(cov)
    
    print(f"质心: {centroid}")
    print(f"主方向特征值: {eigenvalues}")
    
    # 6. 创建不同的显示选项
    print(f"\n=== 尝试不同显示方式 ===")
    
    # 选项1: 原始显示
    print("1. 原始显示")
    o3d.visualization.draw_geometries([pcd], window_name="原始显示")
    
    # 选项2: 带坐标系的显示
    print("2. 带坐标系显示")
    coord = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
    o3d.visualization.draw_geometries([pcd, coord], window_name="带坐标系")
    
    # 选项3: 归一化到原点
    print("3. 归一化到原点")
    pcd_centered = o3d.geometry.PointCloud()
    pcd_centered.points = o3d.utility.Vector3dVector(centered)
    if pcd.has_colors():
        pcd_centered.colors = pcd.colors
    o3d.visualization.draw_geometries([pcd_centered, coord], window_name="归一化")
    
    # 选项4: 如果是毫米单位，转换为米
    if np.max(np.abs(points)) > 10:
        print("4. 毫米转米（如果单位是毫米）")
        points_m = points / 1000.0
        pcd_m = o3d.geometry.PointCloud()
        pcd_m.points = o3d.utility.Vector3dVector(points_m)
        if pcd.has_colors():
            pcd_m.colors = pcd.colors
        o3d.visualization.draw_geometries([pcd_m, coord], window_name="米单位")
    
    # 选项5: 不同的上方向
    print("5. 尝试不同上方向")
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="调整视角")
    vis.add_geometry(pcd)
    
    ctr = vis.get_view_control()
    
    # 尝试几个不同的上方向
    up_vectors = [
        [0, 0, 1],   # Z向上
        [0, 1, 0],   # Y向上  
        [1, 0, 0],   # X向上
        [0, 0, -1],  # Z向下
    ]
    
    for up in up_vectors:
        ctr.set_up(up)
        ctr.set_front([-1, -1, -1])
        ctr.set_lookat(centroid)
        ctr.set_zoom(0.8)
        
        print(f"  上方向: {up}")
        input("按回车尝试下一个视角...")
    
    vis.destroy_window()

# 使用
diagnose_point_cloud("/home/zzz/ros2_ws/src/Camera_example/CPP/build/pointclouds/20251226_213139_986.pcd")