import open3d as o3d
import numpy as np

# 尝试读取PCD文件
try:
    pcd = o3d.io.read_point_cloud("/home/zzz/ros2_ws/src/Camera_example/CPP/build/pointclouds/20251226_213139_986.pcd")
    print(f"成功读取点云")
    print(f"点数: {len(pcd.points)}")
    print(f"点云数据类型: {type(pcd.points)}")
    
    if len(pcd.points) > 0:
        # 显示前5个点
        points = np.asarray(pcd.points)
        print("前5个点坐标:")
        print(points[:5])
        
        # 如果有颜色信息
        if pcd.has_colors():
            colors = np.asarray(pcd.colors)
            print("前5个点颜色:")
            print(colors[:5])
    else:
        print("警告: 点云为空!")
        
except Exception as e:
    print(f"读取失败: {e}")

# 直接查看文件内容
print("\n=== 文件原始内容 (前1000字符) ===")
with open("your_file.pcd", "r") as f:
    content = f.read(1000)
    print(content)