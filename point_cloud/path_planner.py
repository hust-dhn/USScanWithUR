#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import open3d as o3d
from scipy.interpolate import splprep, splev
from scipy.spatial import KDTree
import networkx as nx
import os
import time

class InteractivePathApp:
    def __init__(self, pcd_path, voxel_size=0.002):
        print("1. 正在初始化系统...")
        self.voxel_size = voxel_size
        self.pcd_path = pcd_path
        
        # --- 核心设置：远离表面的距离 (单位: 米) ---
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
        
        print("2. 构建拓扑图 (请稍候)...")
        self.graph = self._build_graph()
        print("   系统就绪！")
        
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
        
        # 全局定向：假设脸部朝向 Z 轴正方向
        pcd.orient_normals_towards_camera_location(pcd.get_center() + np.array([0, 0, 100]))
        
        return pcd

    def _build_graph(self, k_neighbors=15):
        dists, indices = self.kdtree.query(self.points, k=k_neighbors)
        G = nx.Graph()
        for i in range(len(self.points)):
            for j, dist in zip(indices[i], dists[i]):
                if i != j:
                    G.add_edge(i, j, weight=dist)
        return G

    def _compute_smooth_normal(self, point):
        """K-近邻平滑法向"""
        _, indices = self.kdtree.query(point, k=15)
        nearby_normals = np.asarray(self.pcd.normals)[indices]
        avg_normal = np.mean(nearby_normals, axis=0)
        norm = np.linalg.norm(avg_normal)
        if norm > 0:
            avg_normal /= norm
        return avg_normal

    def _generate_smooth_path(self, start_idx, end_idx, num_output_points=300):
        try:
            # 1. Dijkstra 骨架
            path_indices = nx.shortest_path(self.graph, source=start_idx, target=end_idx, weight='weight')
            raw_path = self.points[path_indices]
            
            if len(raw_path) < 5: return None, None

            # 2. 降采样控制点
            step = max(1, len(raw_path) // 15)
            control_points = raw_path[::step]
            if not np.array_equal(control_points[-1], raw_path[-1]):
                control_points = np.vstack([control_points, raw_path[-1]])
            
            # 去重
            clean_cp = [control_points[0]]
            for i in range(1, len(control_points)):
                if np.linalg.norm(control_points[i] - clean_cp[-1]) > 0.005:
                    clean_cp.append(control_points[i])
            control_points = np.array(clean_cp)

            # 3. B样条生成表面上的坐标 (surface_points)
            tck, u = splprep(control_points.T, k=min(3, len(control_points)-1), s=0.0)
            u_new = np.linspace(0, 1, num_output_points)
            surface_points = np.array(splev(u_new, tck)).T
            
            # 4. 计算稳健法向
            temp_normals = []
            for p in surface_points:
                n = self._compute_smooth_normal(p)
                temp_normals.append(n)
            temp_normals = np.array(temp_normals)

            # 连贯性检查
            final_normals = []
            n0 = temp_normals[0]
            vector_from_center = surface_points[0] - self.cloud_center
            if np.dot(n0, vector_from_center) < 0: n0 = -n0 
            
            final_normals.append(n0)
            current_ref = n0
            for i in range(1, len(temp_normals)):
                candidate = temp_normals[i]
                if np.dot(current_ref, candidate) < 0: candidate = -candidate
                final_normals.append(candidate)
                current_ref = candidate
            final_normals = np.array(final_normals)

            # 5. --- 核心逻辑：构建包含“贴合点”的完整轨迹 ---
            
            # 计算悬浮段 (原本的 1cm 路径)
            offset_points = surface_points + final_normals * self.standoff_distance
            
            # 拼接最终轨迹：[表面起点] + [悬浮路径] + [表面终点]
            full_path = np.vstack([
                surface_points[0],    # 1. 起点贴合表面
                offset_points,        # 2. 中间段全部悬浮 1cm
                surface_points[-1]    # 3. 终点贴合表面
            ])
            
            # 对应的法向量也要补全 (首尾各复制一个)
            full_normals = np.vstack([
                final_normals[0],     # 对应表面起点
                final_normals,        # 对应悬浮段
                final_normals[-1]     # 对应表面终点
            ])
            
            return full_path, full_normals

        except Exception as e:
            print(f"路径生成警告: {e}")
            return None, None

    def _calculate_rotation_matrix(self, pos, normal, target_pos):
        """
        构建旋转矩阵 (严格按照 Z -> X -> Y 顺序):
        1. Z轴: 确定为表面法向 (Normal)
        2. X轴: 尽可能指向 Target，但必须投影到 Z 的垂面上
        3. Y轴: 由 Z cross X 确定
        """
        # --- 第一步：确定 Z 轴 ---
        z_axis = normal / (np.linalg.norm(normal) + 1e-9)
        
        # --- 第二步：确定 X 轴 ---
        # 1. 获取原始指向向量 V = Target - Pos
        vec_to_target = target_pos - pos
        
        # 2. 将 V 投影到垂直于 Z 的平面上
        # 公式: V_proj = V - (V dot Z) * Z
        # 这样能保证新的 X 与 Z 严格垂直
        projection = vec_to_target - np.dot(vec_to_target, z_axis) * z_axis
        
        # 3. 处理奇异情况 (如果 Target 恰好在 Z 轴方向上，投影为0)
        if np.linalg.norm(projection) < 1e-6:
            # 这种情况下 X 轴方向不确定，选取一个任意垂直于 Z 的向量
            temp_ref = np.array([1.0, 0.0, 0.0])
            if abs(z_axis[0]) > 0.9: temp_ref = np.array([0.0, 1.0, 0.0])
            x_axis = np.cross(temp_ref, z_axis) # 此时还没定方向，只求垂直
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
        mat[0:3, 0] = x_axis  # R11, R21, R31
        mat[0:3, 1] = y_axis  # R12, R22, R32
        mat[0:3, 2] = z_axis  # R13, R23, R33
        mat[0:3, 3] = pos     # Tx, Ty, Tz
        return mat

    def _create_axes_visualization(self, points, normals, target_pos, axis_len=0.005):
        """生成所有点的坐标轴可视化 geometry"""
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
            
            # 添加点
            lines_points.extend([p0, px, p0, py, p0, pz])
            
            # 添加线索引
            base = idx_counter
            lines_indices.extend([[base, base+1], [base+2, base+3], [base+4, base+5]])
            
            # 添加颜色 (R=X, G=Y, B=Z)
            colors.extend([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
            
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
        print("  3. 姿态定向目标 (X轴正方向将指向此点)")
        print("="*50)
        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window(window_name="1. 选点模式 (需选3点)", width=1000, height=800)
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
        print(f"  路径点数: {len(points)}")
        print("  操作说明:")
        print("  [T] 显示/隐藏 变换矩阵坐标轴 (红X 绿Y 蓝Z)")
        print("  [R] 重置视图")
        print("  [S] 保存矩阵到文件")
        print("  [Q] 退出程序")
        print("="*50)

        vis = o3d.visualization.VisualizerWithKeyCallback()
        vis.create_window(window_name=f"2. 结果预览", width=1000, height=800)

        # 1. 基础几何体
        pcd_display = o3d.geometry.PointCloud(self.pcd)
        pcd_display.paint_uniform_color([0.8, 0.8, 0.8])
        vis.add_geometry(pcd_display)

        # 路径点
        path_pcd = o3d.geometry.PointCloud()
        path_pcd.points = o3d.utility.Vector3dVector(points)
        path_pcd.paint_uniform_color([0.8, 0.1, 0.8])
        vis.add_geometry(path_pcd)

        # 路径线
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(points)
        lines = [[i, i+1] for i in range(len(points)-1)]
        line_set.lines = o3d.utility.Vector2iVector(lines)
        line_set.paint_uniform_color([0.6, 0, 0.6])
        vis.add_geometry(line_set)
        
        # 2. 定向目标可视化 (小蓝球 - 调小尺寸)
        target_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.004) # 原 0.015
        target_sphere.translate(target_pos)
        target_sphere.paint_uniform_color([0, 0, 1])
        vis.add_geometry(target_sphere)

        # 3. 起终点 (绿起 红终 - 调小尺寸)
        for idx, c in zip([start_idx, end_idx], [[0,1,0], [1,0,0]]):
            s_surf = o3d.geometry.TriangleMesh.create_sphere(radius=0.002) # 原 0.005
            s_surf.translate(self.points[idx])
            s_surf.paint_uniform_color(c)
            vis.add_geometry(s_surf)

        # 4. 预计算坐标轴显示 (默认不显示)
        self.axes_geometry = self._create_axes_visualization(points, normals, target_pos)
        self.show_axes_state = False

        # --- 回调函数 ---

        def toggle_axes_callback(vis):
            if self.show_axes_state:
                vis.remove_geometry(self.axes_geometry, reset_bounding_box=False)
                self.show_axes_state = False
                print(" -> 坐标轴: 隐藏")
            else:
                vis.add_geometry(self.axes_geometry, reset_bounding_box=False)
                self.show_axes_state = True
                print(" -> 坐标轴: 显示")
            return True 

        def save_callback(vis):
            filename = f"point_cloud/path_matrix.txt"
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
            print(f">>> 变换矩阵已保存: {filename}")
            return False

        def quit_callback(vis):
            self.keep_running = False
            vis.close()
            return False

        def reset_callback(vis):
            vis.close()
            return False

        # 注册按键
        vis.register_key_callback(ord('R'), reset_callback)
        vis.register_key_callback(ord('S'), save_callback)
        vis.register_key_callback(ord('Q'), quit_callback)
        vis.register_key_callback(ord('T'), toggle_axes_callback)
        
        vis.run()
        vis.destroy_window()

    def run(self):
        while self.keep_running:
            indices = self.run_selection_window()
            
            # 检查选点数量
            if len(indices) < 3:
                print(f"错误: 必须选择 3 个点 (当前选了 {len(indices)} 个)")
                check = input("按回车重试，输入 'q' 退出: ")
                if check.strip().lower() == 'q': break
                continue
            
            # 解析选点
            start_idx = indices[-3]
            end_idx = indices[-2]
            target_idx = indices[-1]
            target_pos = self.points[target_idx]
            
            print(f"路径: {start_idx} -> {end_idx}")
            print(f"定向目标点索引: {target_idx}")
            
            points, normals = self._generate_smooth_path(start_idx, end_idx)
            
            if points is None:
                print("路径生成失败，请重新选点")
                time.sleep(1)
                continue

            self.run_result_window(points, normals, start_idx, end_idx, target_pos)
        print("程序结束")

if __name__ == "__main__":
    PCD_FILE = "point_cloud/target_crop.pcd" 
    if os.path.exists(PCD_FILE):
        InteractivePathApp(PCD_FILE).run()
    else:
        print(f"未找到文件: {PCD_FILE}")