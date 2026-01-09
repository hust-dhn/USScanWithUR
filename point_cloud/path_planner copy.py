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

    def run_selection_window(self):
        print("\n" + "="*50)
        print("【阶段一：选点】 Shift + 左键点击 2 个点")
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

    def run_result_window(self, points, normals, start_idx, end_idx):
        print("\n" + "="*50)
        print("【阶段二：结果预览】")
        print(f"  路径点数: {len(points)}")
        print(f"  偏移距离: {self.standoff_distance}m (悬浮路径)")
        print("  [R]重置  [S]保存(4x4矩阵)  [Q]退出")
        print("="*50)

        vis = o3d.visualization.VisualizerWithKeyCallback()
        vis.create_window(window_name=f"2. 结果 (悬浮 {self.standoff_distance}m)", width=1000, height=800)

        # 1. 原始点云 (灰色)
        pcd_display = o3d.geometry.PointCloud(self.pcd)
        pcd_display.paint_uniform_color([0.8, 0.8, 0.8])
        vis.add_geometry(pcd_display)

        # 2. 悬浮路径点 (紫色)
        path_pcd = o3d.geometry.PointCloud()
        path_pcd.points = o3d.utility.Vector3dVector(points)
        path_pcd.paint_uniform_color([0.8, 0.1, 0.8])
        vis.add_geometry(path_pcd)

        # 3. 路径连线 (深紫)
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(points)
        lines = [[i, i+1] for i in range(len(points)-1)]
        line_set.lines = o3d.utility.Vector2iVector(lines)
        line_set.paint_uniform_color([0.6, 0, 0.6])
        vis.add_geometry(line_set)
        
        # 4. 法向可视化 (橙色)
        normal_length = 0.015 
        normal_points = []
        normal_lines = []
        for i, (p, n) in enumerate(zip(points, normals)):
            normal_points.append(p)
            normal_points.append(p + n * normal_length)
            normal_lines.append([i*2, i*2+1])
            
        normal_vis = o3d.geometry.LineSet()
        normal_vis.points = o3d.utility.Vector3dVector(normal_points)
        normal_vis.lines = o3d.utility.Vector2iVector(normal_lines)
        normal_vis.paint_uniform_color([1.0, 0.6, 0.0])
        vis.add_geometry(normal_vis)

        # 5. 起终点
        for idx, c in zip([start_idx, end_idx], [[0,1,0], [1,0,0]]):
            s_surf = o3d.geometry.TriangleMesh.create_sphere(radius=0.005)
            s_surf.translate(self.points[idx])
            s_surf.paint_uniform_color(c)
            vis.add_geometry(s_surf)

        def reset_callback(vis):
            vis.close()
            return False

        def get_rotation_matrix(pos, normal, tangent):
            """
            根据位置、法向(Z轴)和切线(X轴参考)构建4x4矩阵
            """
            # Z轴: 法线
            z_axis = normal / (np.linalg.norm(normal) + 1e-6)
            
            # X轴: 初始为切线方向
            x_axis = tangent / (np.linalg.norm(tangent) + 1e-6)
            
            # Y轴: Z cross X (建立正交Y)
            y_axis = np.cross(z_axis, x_axis)
            y_axis = y_axis / (np.linalg.norm(y_axis) + 1e-6)
            
            # 修正 X轴: Y cross Z (确保完全正交)
            x_axis = np.cross(y_axis, z_axis)
            x_axis = x_axis / (np.linalg.norm(x_axis) + 1e-6)
            
            # 构建矩阵
            mat = np.eye(4)
            mat[0:3, 0] = x_axis
            mat[0:3, 1] = y_axis
            mat[0:3, 2] = z_axis
            mat[0:3, 3] = pos
            return mat

        def save_callback(vis):
            filename = f"point_cloud/path_matrix.txt"
            
            num_points = len(points)
            matrices = [None] * num_points
            
            # --- 步骤 1: 计算中间点 (Index 1 到 Index N-2) 的矩阵 ---
            # 这里的切线方向取朝向下一个点的方向
            for i in range(1, num_points - 1):
                p_curr = points[i]
                n_curr = normals[i]
                
                # 计算切线：指向下一个点
                tangent = points[i+1] - p_curr
                
                matrices[i] = get_rotation_matrix(p_curr, n_curr, tangent)
            
            # --- 步骤 2: 处理边界条件 (Index 0 和 Index N-1) ---
            # 要求：
            # Index 0 的变换矩阵(旋转) 与 Index 1 保持一致
            # Index Last 的变换矩阵(旋转) 与 Index Last-1 保持一致
            # 注意：必须保留它们自己的位置(Position)
            
            # 处理 Index 0
            mat_0 = matrices[1].copy() # 复制 Index 1 的矩阵
            mat_0[0:3, 3] = points[0]  # 覆盖回 Index 0 的位置
            matrices[0] = mat_0
            
            # 处理 Index Last (-1)
            mat_last = matrices[-2].copy() # 复制 Index -2 的矩阵
            mat_last[0:3, 3] = points[-1]  # 覆盖回 Index -1 的位置
            matrices[-1] = mat_last
            
            # --- 步骤 3: 写入文件 ---
            with open(filename, 'w') as f:
                # 写入表头，说明每列含义
                # 格式: Row-major flattened matrix (r11 r12 r13 tx ...)
                header = "#r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz 0 0 0 1\n"
                f.write(header)
                
                for i, mat in enumerate(matrices):
                    # 展平矩阵的前3行 (最后一行 0 0 0 1 通常省略或写死，这里全部写出)
                    row1 = mat[0, :]
                    row2 = mat[1, :]
                    row3 = mat[2, :]
                    row4 = mat[3, :]
                    
                    line_str = (f"{row1[0]:.6f} {row1[1]:.6f} {row1[2]:.6f} {row1[3]:.6f} "
                                f"{row2[0]:.6f} {row2[1]:.6f} {row2[2]:.6f} {row2[3]:.6f} "
                                f"{row3[0]:.6f} {row3[1]:.6f} {row3[2]:.6f} {row3[3]:.6f} "
                                f"{row4[0]:.0f} {row4[1]:.0f} {row4[2]:.0f} {row4[3]:.0f}\n")
                    f.write(line_str)
                    
            print(f">>> 变换矩阵已保存: {filename}")
            print("    (格式: Index + 4x4 Flattened Matrix)")
            return False

        def quit_callback(vis):
            self.keep_running = False
            vis.close()
            return False

        vis.register_key_callback(ord('R'), reset_callback)
        vis.register_key_callback(ord('S'), save_callback)
        vis.register_key_callback(ord('Q'), quit_callback)
        vis.run()
        vis.destroy_window()

    def run(self):
        while self.keep_running:
            indices = self.run_selection_window()
            if len(indices) < 2:
                check = input("未选够点，按回车重试 (q退出): ")
                if check.strip().lower() == 'q': break
                continue
            
            start, end = indices[-2], indices[-1]
            print(f"计算路径: {start} -> {end}")
            
            points, normals = self._generate_smooth_path(start, end)
            
            if points is None:
                print("路径失败")
                time.sleep(1)
                continue

            self.run_result_window(points, normals, start, end)
        print("程序结束")

if __name__ == "__main__":
    PCD_FILE = "point_cloud/target_crop.pcd" 
    if os.path.exists(PCD_FILE):
        InteractivePathApp(PCD_FILE).run()
    else:
        print("未找到文件")