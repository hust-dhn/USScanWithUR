#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import open3d as o3d
from scipy.spatial import KDTree
import os
import time

class InteractivePathApp:
    def __init__(self, pcd_path, voxel_size=0.002):
        print("1. 正在初始化系统...")
        self.voxel_size = voxel_size
        self.pcd_path = pcd_path
        
        # --- 核心设置：抬升高度 (单位: 米) ---
        self.standoff_distance = 0.01  # 1cm
        
        # 状态变量
        self.show_axes_state = False  # 是否显示坐标轴的开关状态
        self.axes_geometry = None     # 存储坐标轴的LineSet对象
        
        # 加载和预处理
        self.pcd = self._load_pcd(pcd_path, voxel_size)
        self.points = np.asarray(self.pcd.points)
        self.kdtree = KDTree(self.points)
        
        # 计算整个点云的几何中心
        self.cloud_center = np.mean(self.points, axis=0)
        
        # 注意：不再需要构建拓扑图 (Graph)，因为不再搜索路径
        print("   系统就绪！(四点轨迹模式)")
        
        self.keep_running = True

    def _load_pcd(self, path, voxel_size):
        if not os.path.exists(path):
            raise FileNotFoundError(f"文件未找到: {path}")
        pcd = o3d.io.read_point_cloud(path)
        pcd = pcd.voxel_down_sample(voxel_size)
        cl, ind = pcd.remove_statistical_outlier(nb_neighbors=30, std_ratio=1.5)
        pcd = pcd.select_by_index(ind)
        
        if not pcd.has_normals():
            print("   正在估计点云法向...")
            pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=voxel_size * 5, max_nn=30))
        
        # 全局定向：假设脸部/物体朝向 Z 轴正方向
        pcd.orient_normals_towards_camera_location(pcd.get_center() + np.array([0, 0, 100]))
        
        return pcd

    def _compute_smooth_normal(self, point):
        """K-近邻平滑法向，获取该点附近更稳定的法向"""
        _, indices = self.kdtree.query(point, k=15)
        nearby_normals = np.asarray(self.pcd.normals)[indices]
        avg_normal = np.mean(nearby_normals, axis=0)
        norm = np.linalg.norm(avg_normal)
        if norm > 0:
            avg_normal /= norm
        
        # 连贯性检查：确保法向是指向外侧的 (根据点云中心判断)
        # 这里的 normal 是“表面法向”(Pointing OUT)
        vector_from_center = point - self.cloud_center
        if np.dot(avg_normal, vector_from_center) < 0:
            avg_normal = -avg_normal
            
        return avg_normal

    def _generate_four_point_path(self, start_idx, end_idx):
        """
        生成仅包含4个点的轨迹：
        1. 表面起点
        2. 起点抬升
        3. 终点抬升
        4. 表面终点
        """
        try:
            # 1. 获取基础坐标
            p_start_surf = self.points[start_idx]
            p_end_surf = self.points[end_idx]

            # 2. 获取稳健的表面法向
            n_start = self._compute_smooth_normal(p_start_surf)
            n_end = self._compute_smooth_normal(p_end_surf)

            # 3. 计算抬升点
            # 逻辑：沿着法向向外延伸 standoff_distance
            p_start_lift = p_start_surf + (n_start * self.standoff_distance)
            p_end_lift = p_end_surf + (n_end * self.standoff_distance)

            # 4. 组装 4 个点 [StartSurf, StartLift, EndLift, EndSurf]
            full_path = np.vstack([
                p_start_surf,
                p_start_lift,
                p_end_lift,
                p_end_surf
            ])

            # 5. 组装对应的法向
            # P1, P2 使用起点的法向
            # P3, P4 使用终点的法向
            full_normals = np.vstack([
                n_start,
                n_start,
                n_end,
                n_end
            ])
            
            return full_path, full_normals

        except Exception as e:
            print(f"路径生成警告: {e}")
            return None, None

    def _calculate_rotation_matrix(self, pos, normal, target_pos):
        """
        构建旋转矩阵:
        Z轴: 沿法线负方向 (Input Normal 是朝外的，所以 -Normal 朝里)
        X轴: 指向 Target
        Y轴: 右手定则
        """
        # --- 第一步：确定 Z 轴 ---
        # 用户要求: "每个点的z方向要沿着法线负方向"
        # 这里的 normal 已经是朝外的表面法向了，所以取反即为负方向（朝内）
        z_axis = -normal 
        z_axis = z_axis / (np.linalg.norm(z_axis) + 1e-9)
        
        # --- 第二步：确定 X 轴 ---
        # 1. 获取原始指向向量 V = Target - Pos
        vec_to_target = target_pos - pos
        
        # 2. 将 V 投影到垂直于 Z 的平面上 (Gram-Schmidt process)
        # 这样能保证 X 与 Z 严格垂直，且 X 尽可能指向 Target
        projection = vec_to_target - np.dot(vec_to_target, z_axis) * z_axis
        
        # 3. 处理奇异情况
        if np.linalg.norm(projection) < 1e-6:
            temp_ref = np.array([1.0, 0.0, 0.0])
            if abs(z_axis[0]) > 0.9: temp_ref = np.array([0.0, 1.0, 0.0])
            x_axis = np.cross(temp_ref, z_axis)
        else:
            x_axis = projection
            
        # 4. 归一化 X
        x_axis = x_axis / (np.linalg.norm(x_axis) + 1e-9)
        
        # --- 第三步：确定 Y 轴 ---
        # 右手定则: Z cross X = Y
        y_axis = np.cross(z_axis, x_axis)
        y_axis = y_axis / (np.linalg.norm(y_axis) + 1e-9)
        
        # 构建矩阵 (列向量 [X, Y, Z, Pos])
        mat = np.eye(4)
        mat[0:3, 0] = x_axis  # R11, R21, R31 (X)
        mat[0:3, 1] = y_axis  # R12, R22, R32 (Y)
        mat[0:3, 2] = z_axis  # R13, R23, R33 (Z)
        mat[0:3, 3] = pos     # Tx, Ty, Tz
        return mat

    def _create_axes_visualization(self, points, normals, target_pos, axis_len=0.005):
        """生成坐标轴可视化"""
        lines_points = []
        lines_indices = []
        colors = []
        idx_counter = 0
        
        for i in range(len(points)):
            pos = points[i]
            normal = normals[i]
            
            mat = self._calculate_rotation_matrix(pos, normal, target_pos)
            x_axis = mat[0:3, 0]
            y_axis = mat[0:3, 1]
            z_axis = mat[0:3, 2]
            
            # 原点
            p0 = pos
            # 轴的端点
            px = pos + x_axis * axis_len
            py = pos + y_axis * axis_len
            pz = pos + z_axis * axis_len
            
            lines_points.extend([p0, px, p0, py, p0, pz])
            base = idx_counter
            lines_indices.extend([[base, base+1], [base+2, base+3], [base+4, base+5]])
            colors.extend([[1, 0, 0], [0, 1, 0], [0, 0, 1]]) # RGB -> XYZ
            idx_counter += 6
            
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(lines_points)
        line_set.lines = o3d.utility.Vector2iVector(lines_indices)
        line_set.colors = o3d.utility.Vector3dVector(colors)
        return line_set

    def run_selection_window(self):
        print("\n" + "="*50)
        print("【阶段一：选点】 请按顺序 Shift + 左键点击 3 个点")
        print("  1. 路径起点")
        print("  2. 路径终点")
        print("  3. 姿态定向目标 (X轴将指向此点)")
        print("="*50)
        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window(window_name="1. 选点模式", width=1000, height=800)
        pcd_display = o3d.geometry.PointCloud(self.pcd)
        pcd_display.paint_uniform_color([0.7, 0.7, 0.7])
        vis.add_geometry(pcd_display)
        vis.run()
        picked_indices = vis.get_picked_points()
        vis.destroy_window()
        return picked_indices

    def run_result_window(self, points, normals, start_idx, end_idx, target_pos):
        print("\n" + "="*50)
        print("【阶段二：结果预览】")
        print(f"  已生成 {len(points)} 个关键点 (4点模式)")
        print("  操作说明:")
        print("  [T] 显示/隐藏 坐标轴 (红X 绿Y 蓝Z)")
        print("  [S] 保存矩阵到文件")
        print("  [Q] 退出程序")
        print("="*50)

        vis = o3d.visualization.VisualizerWithKeyCallback()
        vis.create_window(window_name=f"2. 结果预览", width=1000, height=800)

        # 1. 基础几何体
        pcd_display = o3d.geometry.PointCloud(self.pcd)
        pcd_display.paint_uniform_color([0.8, 0.8, 0.8])
        vis.add_geometry(pcd_display)

        # 路径点 (关键点)
        path_pcd = o3d.geometry.PointCloud()
        path_pcd.points = o3d.utility.Vector3dVector(points)
        path_pcd.paint_uniform_color([1.0, 0.1, 0.1]) # 红色高亮关键点
        vis.add_geometry(path_pcd)

        # 路径线 (连接这4个点)
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(points)
        lines = [[i, i+1] for i in range(len(points)-1)]
        line_set.lines = o3d.utility.Vector2iVector(lines)
        line_set.paint_uniform_color([0, 0, 1]) # 蓝色连线
        vis.add_geometry(line_set)
        
        # 2. 定向目标可视化
        target_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.004)
        target_sphere.translate(target_pos)
        target_sphere.paint_uniform_color([0, 0, 1])
        vis.add_geometry(target_sphere)

        # 3. 预计算坐标轴
        self.axes_geometry = self._create_axes_visualization(points, normals, target_pos)
        self.show_axes_state = False

        # --- 回调函数 ---
        def toggle_axes_callback(vis):
            if self.show_axes_state:
                vis.remove_geometry(self.axes_geometry, reset_bounding_box=False)
                self.show_axes_state = False
            else:
                vis.add_geometry(self.axes_geometry, reset_bounding_box=False)
                self.show_axes_state = True
            return True 

        def save_callback(vis):
            filename = f"point_cloud/path_matrix_4points.txt"
            with open(filename, 'w') as f:
                header = "#r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz 0 0 0 1\n"
                f.write(header)
                for i in range(len(points)):
                    mat = self._calculate_rotation_matrix(points[i], normals[i], target_pos)
                    # 展平写入
                    row1, row2, row3, row4 = mat[0], mat[1], mat[2], mat[3]
                    line_str = (f"{row1[0]:.6f} {row1[1]:.6f} {row1[2]:.6f} {row1[3]:.6f} "
                                f"{row2[0]:.6f} {row2[1]:.6f} {row2[2]:.6f} {row2[3]:.6f} "
                                f"{row3[0]:.6f} {row3[1]:.6f} {row3[2]:.6f} {row3[3]:.6f} "
                                f"{row4[0]:.0f} {row4[1]:.0f} {row4[2]:.0f} {row4[3]:.0f}\n")
                    f.write(line_str)
            print(f">>> 4点变换矩阵已保存: {filename}")
            return False

        def quit_callback(vis):
            self.keep_running = False
            vis.close()
            return False

        def reset_callback(vis):
            vis.close()
            return False

        vis.register_key_callback(ord('R'), reset_callback)
        vis.register_key_callback(ord('S'), save_callback)
        vis.register_key_callback(ord('Q'), quit_callback)
        vis.register_key_callback(ord('T'), toggle_axes_callback)
        
        vis.run()
        vis.destroy_window()

    def run(self):
        while self.keep_running:
            indices = self.run_selection_window()
            
            if len(indices) < 3:
                print(f"错误: 必须选择 3 个点 (当前选了 {len(indices)} 个)")
                check = input("按回车重试，输入 'q' 退出: ")
                if check.strip().lower() == 'q': break
                continue
            
            start_idx = indices[-3]
            end_idx = indices[-2]
            target_idx = indices[-1]
            target_pos = self.points[target_idx]
            
            print(f"起点索引: {start_idx}, 终点索引: {end_idx}")
            
            # 使用新的4点生成函数
            points, normals = self._generate_four_point_path(start_idx, end_idx)
            
            if points is None:
                print("路径生成失败，请重试")
                continue

            self.run_result_window(points, normals, start_idx, end_idx, target_pos)
        print("程序结束")

if __name__ == "__main__":
    PCD_FILE = "point_cloud/new_target.pcd" 
    if os.path.exists(PCD_FILE):
        InteractivePathApp(PCD_FILE).run()
    else:
        print(f"未找到文件: {PCD_FILE}")