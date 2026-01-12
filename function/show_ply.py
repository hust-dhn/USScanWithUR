import open3d as o3d


PLY_PATH = 'function/pcd/face_m_pc1.ply'
# 1. 读取PLY文件
pcd = o3d.io.read_point_cloud(PLY_PATH)  # 读取点云
# 如果文件是网格，使用: mesh = o3d.io.read_triangle_mesh("your_file.ply")

# 2. 基本可视化
print(pcd)  # 打印点云基本信息（点数、是否有颜色等）
o3d.visualization.draw_geometries([pcd], 
                                  window_name="Open3D点云显示",
                                  width=800,  # 窗口宽度
                                  height=600) # 窗口高度