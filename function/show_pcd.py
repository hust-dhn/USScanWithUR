import open3d as o3d

# 1. 加载
pcd = o3d.io.read_point_cloud("/home/zzz/ros2_ws/src/Camera_example/CPP/build/saved_cloud_xyzrgb.pcd")

# 2. 创建坐标轴（红色=X, 绿色=Y, 蓝色=Z），size=1.0表示轴长1米
axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)

# 3. 显示
print(f"显示点云: {len(pcd.points)} 个点")
o3d.visualization.draw_geometries([pcd, axes], window_name="Open3D Viewer")