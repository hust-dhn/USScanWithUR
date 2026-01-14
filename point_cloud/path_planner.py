#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import open3d as o3d
from scipy.spatial import KDTree
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
import os

class InteractivePathApp:
    def __init__(self, pcd_path, voxel_size=0.002):
        print("1. 正在初始化系统...")
        self.voxel_size = voxel_size
        self.pcd_path = pcd_path
        
        # --- 核心高度设置 ---
        self.working_lift = 0.01        # 起点和终点的实际工作高度 (1cm)
        self.standoff_distance = 0.04   # 中间移动时的抬升高度 (4cm)
        self.interpolation_steps = 100   # 中间过渡的插值步数
        
        self.show_axes_state = False
        self.axes_geometry = None
        
        self.pcd = self._load_pcd(pcd_path, voxel_size)
        self.points = np.asarray(self.pcd.points)
        self.kdtree = KDTree(self.points)
        self.cloud_center = np.mean(self.points, axis=0)
        
        print(f"   系统就绪！高度设定：端点 {self.working_lift*100}cm, 过程中 {self.standoff_distance*100}cm")
        self.keep_running = True

    def _load_pcd(self, path, voxel_size):
        if not os.path.exists(path):
            raise FileNotFoundError(f"文件未找到: {path}")
        pcd = o3d.io.read_point_cloud(path)
        pcd = pcd.voxel_down_sample(voxel_size)
        _, ind = pcd.remove_statistical_outlier(nb_neighbors=30, std_ratio=1.5)
        pcd = pcd.select_by_index(ind)
        if not pcd.has_normals():
            pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=30))
        pcd.orient_normals_towards_camera_location(pcd.get_center() + np.array([0, 0, 100]))
        return pcd

    def _compute_smooth_normal(self, point):
        _, indices = self.kdtree.query(point, k=15)
        nearby_normals = np.asarray(self.pcd.normals)[indices]
        avg_normal = np.mean(nearby_normals, axis=0)
        norm = np.linalg.norm(avg_normal)
        if norm > 0: avg_normal /= norm
        if np.dot(avg_normal, point - self.cloud_center) < 0:
            avg_normal = -avg_normal
        return avg_normal

    def _interpolate_poses(self, mat_start, mat_end, n_steps):
        pos_start, pos_end = mat_start[0:3, 3], mat_end[0:3, 3]
        key_rots = R.from_matrix([mat_start[0:3, 0:3], mat_end[0:3, 0:3]])
        slerp = Slerp([0, 1], key_rots)
        
        interp_mats = []
        ratios = np.linspace(0, 1, n_steps + 2) 
        for ratio in ratios[1:-1]:
            mat = np.eye(4)
            mat[0:3, 0:3] = slerp([ratio]).as_matrix()[0]
            mat[0:3, 3] = (1 - ratio) * pos_start + ratio * pos_end
            interp_mats.append(mat)
        return interp_mats

    def _calculate_rotation_matrix(self, pos, normal, target_pos):
        z_axis = -normal 
        z_axis /= (np.linalg.norm(z_axis) + 1e-9)
        vec_to_target = target_pos - pos
        projection = vec_to_target - np.dot(vec_to_target, z_axis) * z_axis
        if np.linalg.norm(projection) < 1e-6:
            temp_ref = np.array([0.0, 1.0, 0.0]) if abs(z_axis[0]) > 0.9 else np.array([1.0, 0.0, 0.0])
            x_axis = np.cross(temp_ref, z_axis)
        else:
            x_axis = projection / (np.linalg.norm(projection) + 1e-9)
        y_axis = np.cross(z_axis, x_axis)
        y_axis /= (np.linalg.norm(y_axis) + 1e-9)
        
        mat = np.eye(4)
        mat[0:3, 0:3] = np.stack([x_axis, y_axis, z_axis], axis=1)
        mat[0:3, 3] = pos
        return mat

    def _generate_full_trajectory(self, start_idx, end_idx, target_start_pos, target_end_pos):
        # 1. 获取表面点和法向
        p_s_surf = self.points[start_idx]
        p_e_surf = self.points[end_idx]
        n_s = self._compute_smooth_normal(p_s_surf)
        n_e = self._compute_smooth_normal(p_e_surf)
        
        # 2. 计算四个关键位置
        # P1: 起点 1cm 高度
        p1_pos = p_s_surf + (n_s * self.working_lift)
        # P2: 起点 4cm 高度
        p2_pos = p_s_surf + (n_s * self.standoff_distance)
        # P3: 终点 4cm 高度
        p3_pos = p_e_surf + (n_e * self.standoff_distance)
        # P4: 终点 1cm 高度
        p4_pos = p_e_surf + (n_e * self.working_lift)

        # 3. 构造矩阵 (P1,P2 指向 target_start; P3,P4 指向 target_end)
        mat1 = self._calculate_rotation_matrix(p1_pos, n_s, target_start_pos)
        mat2 = self._calculate_rotation_matrix(p2_pos, n_s, target_start_pos)
        mat3 = self._calculate_rotation_matrix(p3_pos, n_e, target_end_pos)
        mat4 = self._calculate_rotation_matrix(p4_pos, n_e, target_end_pos)

        # 4. 在 P2 和 P3 之间（4cm高度层）进行平滑插值
        interp_mats = self._interpolate_poses(mat2, mat3, self.interpolation_steps)

        # 5. 最终轨迹点序列
        all_matrices = [mat1, mat2] + interp_mats + [mat3, mat4]
        all_points = [m[0:3, 3] for m in all_matrices]

        return np.array(all_points), all_matrices

    def _create_axes_visualization(self, matrices, axis_len=0.01):
        lines_points, lines_indices, colors = [], [], []
        for i, mat in enumerate(matrices):
            pos = mat[0:3, 3]
            for j in range(3): # X, Y, Z axes
                lines_points.extend([pos, pos + mat[0:3, j] * axis_len])
                lines_indices.append([len(lines_points)-2, len(lines_points)-1])
                c = [0,0,0]; c[j] = 1
                colors.append(c)
        ls = o3d.geometry.LineSet()
        ls.points = o3d.utility.Vector3dVector(lines_points)
        ls.lines = o3d.utility.Vector2iVector(lines_indices)
        ls.colors = o3d.utility.Vector3dVector(colors)
        return ls

    def run_selection_window(self):
        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window(window_name="步骤1: 选4个点 (起, 终, 起向, 终向)", width=1000, height=800)
        vis.add_geometry(self.pcd)
        print("\n>>> [Shift+左键]选点: 1.起点 2.终点 3.起点朝向 4.终点朝向. 选完按Q")
        vis.run(); indices = vis.get_picked_points(); vis.destroy_window()
        return indices

    def run_result_window(self, points, matrices, t1, t2):
        vis = o3d.visualization.VisualizerWithKeyCallback()
        vis.create_window(window_name="步骤2: 结果预览 (T:坐标轴 S:保存 Q:退出)", width=1000, height=800)
        vis.add_geometry(o3d.geometry.PointCloud(self.pcd).paint_uniform_color([0.8, 0.8, 0.8]))
        
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(points)
        line_set.lines = o3d.utility.Vector2iVector([[i, i+1] for i in range(len(points)-1)])
        line_set.paint_uniform_color([1, 0, 0])
        vis.add_geometry(line_set)

        self.axes_geometry = self._create_axes_visualization(matrices)
        self.show_axes_state = False

        def toggle_axes(vis):
            if self.show_axes_state: vis.remove_geometry(self.axes_geometry, False)
            else: vis.add_geometry(self.axes_geometry, False)
            self.show_axes_state = not self.show_axes_state
            return True 

        def save_mats(vis):
            os.makedirs("point_cloud", exist_ok=True)
            with open("point_cloud/path_matrices.txt", 'w') as f:
                for m in matrices:
                    f.write(" ".join(f"{x:.6f}" for x in m.flatten()) + "\n")
            print(">>> 已保存至 point_cloud/path_matrices.txt")
            return False

        vis.register_key_callback(ord('T'), toggle_axes)
        vis.register_key_callback(ord('S'), save_mats)
        vis.run(); vis.destroy_window()

    def run(self):
        while self.keep_running:
            indices = self.run_selection_window()
            if len(indices) < 4: break
            p_s, p_e = indices[0], indices[1]
            t_s, t_e = self.points[indices[2]], self.points[indices[3]]
            pts, mats = self._generate_full_trajectory(p_s, p_e, t_s, t_e)
            self.run_result_window(pts, mats, t_s, t_e)
            if input("继续？(y/n): ").lower() != 'y': break

if __name__ == "__main__":
    PCD_FILE = "point_cloud/new_target.pcd" 
    if os.path.exists(PCD_FILE): InteractivePathApp(PCD_FILE).run()
    else: print("文件不存在")