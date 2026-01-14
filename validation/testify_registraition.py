import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

def visualize_two_pointclouds(pcd_file1, pcd_file2, color1=None, color2=None):
    """
    加载并可视化两个点云文件
    
    参数:
    pcd_file1: 第一个点云文件路径
    pcd_file2: 第二个点云文件路径
    color1: 第一个点云颜色 [R, G, B], 范围0-1
    color2: 第二个点云颜色 [R, G, B], 范围0-1
    """
    
    # 1. 加载点云文件
    print(f"正在加载点云文件: {pcd_file1}")
    pcd1 = o3d.io.read_point_cloud(pcd_file1)
    
    print(f"正在加载点云文件: {pcd_file2}")
    pcd2 = o3d.io.read_point_cloud(pcd_file2)
    
    # 2. 打印点云信息
    print("\n点云1信息:")
    print(f"  点数: {len(pcd1.points)}")
    print(f"  是否有颜色: {pcd1.has_colors()}")
    print(f"  是否有法线: {pcd1.has_normals()}")
    
    print("\n点云2信息:")
    print(f"  点数: {len(pcd2.points)}")
    print(f"  是否有颜色: {pcd2.has_colors()}")
    print(f"  是否有法线: {pcd2.has_normals()}")
    
    # 3. 设置点云颜色（如果没有颜色）
    if not pcd1.has_colors():
        if color1 is None:
            color1 = [1, 0, 0]  # 红色
        pcd1.paint_uniform_color(color1)
    
    if not pcd2.has_colors():
        if color2 is None:
            color2 = [0, 0, 1]  # 蓝色
        pcd2.paint_uniform_color(color2)
    
    # 4. 可视化（同一坐标系）
    print("\n正在打开可视化窗口...")
    print("窗口控制:")
    print("  - 鼠标左键: 旋转视角")
    print("  - 鼠标右键: 平移")
    print("  - 滚轮: 缩放")
    print("  - 'L': 打开/关闭灯光")
    print("  - 'R': 重置视角")
    print("  - 'Q': 退出")
    
    # 创建可视化窗口
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="双点云可视化", width=1024, height=768)
    
    # 添加点云到可视化器
    vis.add_geometry(pcd1)
    vis.add_geometry(pcd2)
    
    # 设置渲染选项
    opt = vis.get_render_option()
    opt.background_color = np.asarray([0.1, 0.1, 0.1])  # 深灰色背景
    opt.point_size = 2.0  # 点大小
    opt.show_coordinate_frame = True  # 显示坐标系
    
    # 运行可视化
    vis.run()
    vis.destroy_window()

def visualize_with_matplotlib(pcd_file1, pcd_file2):
    """
    使用Matplotlib可视化（2D投影）
    """
    # 加载点云
    pcd1 = o3d.io.read_point_cloud(pcd_file1)
    pcd2 = o3d.io.read_point_cloud(pcd_file2)
    
    # 转换为numpy数组
    points1 = np.asarray(pcd1.points)
    points2 = np.asarray(pcd2.points)
    
    # 创建子图
    fig = plt.figure(figsize=(15, 5))
    
    # 1. XY平面投影
    ax1 = fig.add_subplot(131)
    ax1.scatter(points1[:, 0], points1[:, 1], c='r', s=1, alpha=0.5, label='点云1')
    ax1.scatter(points2[:, 0], points2[:, 1], c='b', s=1, alpha=0.5, label='点云2')
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_title('XY平面')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # 2. XZ平面投影
    ax2 = fig.add_subplot(132)
    ax2.scatter(points1[:, 0], points1[:, 2], c='r', s=1, alpha=0.5, label='点云1')
    ax2.scatter(points2[:, 0], points2[:, 2], c='b', s=1, alpha=0.5, label='点云2')
    ax2.set_xlabel('X')
    ax2.set_ylabel('Z')
    ax2.set_title('XZ平面')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # 3. YZ平面投影
    ax3 = fig.add_subplot(133)
    ax3.scatter(points1[:, 1], points1[:, 2], c='r', s=1, alpha=0.5, label='点云1')
    ax3.scatter(points2[:, 1], points2[:, 2], c='b', s=1, alpha=0.5, label='点云2')
    ax3.set_xlabel('Y')
    ax3.set_ylabel('Z')
    ax3.set_title('YZ平面')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    plt.suptitle('双点云多视角可视化', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.show()

def save_visualization_image(pcd_file1, pcd_file2, output_file="pointclouds.png"):
    """
    保存可视化图像
    """
    # 加载点云
    pcd1 = o3d.io.read_point_cloud(pcd_file1)
    pcd2 = o3d.io.read_point_cloud(pcd_file2)
    
    # 设置颜色
    if not pcd1.has_colors():
        pcd1.paint_uniform_color([1, 0, 0])  # 红色
    
    if not pcd2.has_colors():
        pcd2.paint_uniform_color([0, 0, 1])  # 蓝色
    
    # 创建可视化器
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False)  # 不可见窗口
    vis.add_geometry(pcd1)
    vis.add_geometry(pcd2)
    
    # 设置视角
    ctr = vis.get_view_control()
    ctr.set_front([0, 0, -1])  # 设置相机朝向
    ctr.set_up([0, -1, 0])     # 设置相机上方向
    ctr.set_zoom(0.8)          # 缩放
    
    # 渲染并保存
    vis.poll_events()
    vis.update_renderer()
    vis.capture_screen_image(output_file, do_render=True)
    vis.destroy_window()
    
    print(f"可视化图像已保存到: {output_file}")

if __name__ == "__main__":
    # 设置你的PCD文件路径
    pcd_file1 = "/home/zzz/ros2_ws/src/Camera_example/CPP/app/PCLProject/pcd_data/global_raw_pointcloud.pcd"  # 替换为你的第一个PCD文件路径
    pcd_file2 = "/home/zzz/ros2_ws/src/Camera_example/CPP/app/PCLProject/debug_pcd/transformed_source.pcd"  # 替换为你的第二个PCD文件路径
    
    try:
        # 方法1: 使用Open3D交互式可视化（推荐）
        print("=" * 50)
        print("方法1: Open3D交互式可视化")
        print("=" * 50)
        visualize_two_pointclouds(
            pcd_file1, 
            pcd_file2,
            color1=[1, 0, 0],  # 红色
            color2=[0, 1, 0]   # 绿色
        )
        
        # 方法2: 使用Matplotlib进行2D投影可视化
        print("\n" + "=" * 50)
        print("方法2: Matplotlib 2D投影可视化")
        print("=" * 50)
        visualize_with_matplotlib(pcd_file1, pcd_file2)
        
        # 方法3: 保存可视化图像
        print("\n" + "=" * 50)
        print("方法3: 保存可视化图像")
        print("=" * 50)
        save_visualization_image(pcd_file1, pcd_file2, "combined_pointclouds.png")
        
    except FileNotFoundError as e:
        print(f"错误: 找不到文件 - {e}")
        print("请确保文件路径正确，并检查文件是否存在")
    except Exception as e:
        print(f"发生错误: {e}")
        print("请检查PCD文件格式是否正确")