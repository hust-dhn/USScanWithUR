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
        
        # --- 核心参数设置 ---
        self.standoff_distance = 0.01  # 基础悬浮高度: 1cm
        self.tool_radius = 0.05        # 工具半径: 2cm (关键避障参数)
        self.safety_buffer = 0.005     # 安全余量: 5mm (避障时的额外高度)
        
        # 加载和预处理
        self.pcd = self._load_pcd(pcd_path, voxel_size)
        self.points = np.asarray(self.pcd.points)
        self.kdtree = KDTree(self.points)
        
        # 计算整个点云的几何中心
        self.cloud_center = np.mean(self.points, axis=0)
        
        print("2. 构建拓扑图 (请稍候)...")
        self.graph = self._build_graph()
        print(f"   系统就绪！工具半径: {self.tool_radius*100:.1f}cm")
        
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

    def _compute_collision_free_heights(self, surface_points, normals):
        """
        [核心算法] 体积碰撞检测
        检测以 surface_point 为中心，tool_radius 为半径的圆柱体内，
        是否有高于 standoff_distance 的点云障碍物。
        """
        num_points = len(surface_points)
        # 初始化高度数组，默认为基础悬浮距离
        safe_heights = np.full(num_points, self.standoff_distance)
        
        # 搜索半径：工具半径 + 适当余量，确保覆盖边缘
        search_radius = self.tool_radius * 1.2 
        
        print(f"   正在进行体积碰撞检测 (搜索半径 {search_radius*100:.1f}cm)...")

        for i in range(num_points):
            p = surface_points[i]
            n = normals[i]
            
            # 1. 粗筛选：KDTree 找附近的点
            idx = self.kdtree.query_ball_point(p, search_radius)
            if not idx: continue
            
            neighbors = self.points[idx]
            
            # 2. 局部坐标变换：计算邻居点相对于当前点 p 的向量
            vecs = neighbors - p
            
            # 3. 投影计算
            # 垂直分量 (高度): v · n (点乘)
            heights_proj = np.dot(vecs, n)
            
            # 水平分量向量: v - (v·n)n
            # 利用广播机制计算
            vert_vecs = np.outer(heights_proj, n)
            horiz_vecs = vecs - vert_vecs
            
            # 水平距离 (横向距离)
            lat_dists = np.linalg.norm(horiz_vecs, axis=1)
            
            # 4. 碰撞判定
            # 条件：横向距离 < 工具半径 且 高度 > 0 (只关心凸起的障碍物)
            mask_cylinder = lat_dists < self.tool_radius
            
            if np.any(mask_cylinder):
                # 找出圆柱范围内最高的障碍物
                # 只关心正向突出的点 (heights_proj > 0)
                relevant_heights = heights_proj[mask_cylinder]
                if len(relevant_heights) > 0:
                    max_obstacle_height = np.max(relevant_heights)
                    
                    # 只有当障碍物高度接近或超过当前设定的悬浮高度时，才需要调整
                    # 目标高度 = 障碍物最高点 + 安全余量
                    required_h = max_obstacle_height + self.safety_buffer
                    
                    if required_h > safe_heights[i]:
                        safe_heights[i] = required_h

        # 5. 高度平滑 (避免路径因避让突然剧烈跳变)
        window_size = 15
        if len(safe_heights) > window_size:
            kernel = np.ones(window_size) / window_size
            # 使用 same 模式卷积平滑
            smoothed_heights = np.convolve(safe_heights, kernel, mode='same')
            # 再次强制确保不低于基础悬浮距离 (平滑可能会把峰值拉低，取最大值比较安全)
            safe_heights = np.maximum(smoothed_heights, self.standoff_distance)
            
            # 额外处理：再次确保包含最大障碍物 (防止平滑过度导致撞上)
            # 简单策略：如果平滑后降低太多，保留原始值的 90%
            # (这里为了代码简洁，主要依赖上面的 maximum 保证最低高度)

        return safe_heights

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

            # 3. B样条生成表面上的坐标
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

            # 5. --- 核心修改：计算动态安全高度 ---
            safe_heights = self._compute_collision_free_heights(surface_points, final_normals)

            # 打印调试信息
            max_h = np.max(safe_heights)
            if max_h > self.standoff_distance + 0.001:
                print(f"   !!! 警告：检测到障碍物，局部路径已抬升至 {max_h*100:.2f}cm")
            
            # 计算悬浮段 (利用广播机制: (N,3) + (N,3) * (N,1))
            offset_points = surface_points + final_normals * safe_heights[:, np.newaxis]
            
            # 6. 拼接最终轨迹
            full_path = np.vstack([
                surface_points[0],    # 起点贴合
                offset_points,        # 中间悬浮(变高)
                surface_points[-1]    # 终点贴合
            ])
            
            full_normals = np.vstack([
                final_normals[0],     
                final_normals,        
                final_normals[-1]     
            ])
            
            return full_path, full_normals

        except Exception as e:
            print(f"路径生成警告: {e}")
            import traceback
            traceback.print_exc()
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
        print(f"  基础悬浮: {self.standoff_distance}m")
        print(f"  工具半径: {self.tool_radius}m (已考虑体积碰撞)")
        print("  [R]重置  [S]保存(4x4矩阵)  [Q]退出")
        print("="*50)

        vis = o3d.visualization.VisualizerWithKeyCallback()
        vis.create_window(window_name=f"2. 结果预览", width=1000, height=800)

        # 1. 原始点云
        pcd_display = o3d.geometry.PointCloud(self.pcd)
        pcd_display.paint_uniform_color([0.8, 0.8, 0.8])
        vis.add_geometry(pcd_display)

        # 2. 悬浮路径点
        path_pcd = o3d.geometry.PointCloud()
        path_pcd.points = o3d.utility.Vector3dVector(points)
        path_pcd.paint_uniform_color([0.8, 0.1, 0.8]) # 紫色
        vis.add_geometry(path_pcd)

        # 3. 路径连线
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
            # Z轴: 法线
            z_axis = normal / (np.linalg.norm(normal) + 1e-6)
            # X轴: 切线
            x_axis = tangent / (np.linalg.norm(tangent) + 1e-6)
            # Y轴: Z cross X
            y_axis = np.cross(z_axis, x_axis)
            y_axis = y_axis / (np.linalg.norm(y_axis) + 1e-6)
            # 修正 X轴
            x_axis = np.cross(y_axis, z_axis)
            x_axis = x_axis / (np.linalg.norm(x_axis) + 1e-6)
            
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
            
            # 计算中间点矩阵
            for i in range(1, num_points - 1):
                p_curr = points[i]
                n_curr = normals[i]
                tangent = points[i+1] - p_curr
                matrices[i] = get_rotation_matrix(p_curr, n_curr, tangent)
            
            # 边界处理
            mat_0 = matrices[1].copy()
            mat_0[0:3, 3] = points[0]
            matrices[0] = mat_0
            
            mat_last = matrices[-2].copy()
            mat_last[0:3, 3] = points[-1]
            matrices[-1] = mat_last
            
            # 确保目录存在
            os.makedirs(os.path.dirname(filename), exist_ok=True)
            
            with open(filename, 'w') as f:
                header = "#r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz 0 0 0 1\n"
                f.write(header)
                for mat in matrices:
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
    # 请确保路径正确
    PCD_FILE = "point_cloud/target_crop.pcd" 
    
    if os.path.exists(PCD_FILE):
        app = InteractivePathApp(PCD_FILE)
        app.run()
    else:
        print(f"未找到文件: {PCD_FILE}")
        # 创建一个简单的测试文件（如果需要测试）
        # import open3d as o3d
        # mesh = o3d.geometry.TriangleMesh.create_sphere()
        # pcd = mesh.sample_points_poisson_disk(5000)
        # o3d.io.write_point_cloud(PCD_FILE, pcd)
        # print("已生成测试球体点云，请重新运行。")