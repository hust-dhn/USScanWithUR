#!/usr/bin/env python3
"""
手眼标定程序 (Eye-in-Hand) - 独立版本
用于计算相机到机械臂末端的变换矩阵 T_cam2gripper

使用说明:
1. 准备机器人位姿文件 (robot_pos.txt) - 4x4矩阵格式
2. 准备棋盘格图像文件夹 (rc_imgs/)
3. 运行: python hand_eye_calibration_standalone.py
"""

import numpy as np
import cv2
import yaml
import glob
import os
import sys
import math
from scipy.spatial.transform import Rotation as R
from datetime import datetime


class HandEyeCalibration:
    """
    手眼标定类，实现相机到机械臂末端的外参标定
    """
    
    def __init__(self, camera_matrix, dist_coeffs, pattern_size, square_size):
        """
        初始化手眼标定类
        
        参数:
            camera_matrix: 相机内参矩阵 (3x3)
            dist_coeffs: 畸变系数
            pattern_size: 棋盘格内部角点数量 (cols-1, rows-1)
            square_size: 棋盘格方格边长 (米)
        """
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.pattern_size = pattern_size
        self.square_size = square_size
        
        # 生成标定板3D坐标 (世界坐标系，假设棋盘格在XY平面)
        self.objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
        self.objp *= square_size
        
        print(f"棋盘格参数: {pattern_size} 个内部角点，方格大小: {square_size}米")
        
    def load_robot_poses(self, pose_file):
        """
        加载机器人末端位姿文件
        
        格式: 每行16个数字，表示4x4变换矩阵 (按行展开)
        
        返回:
            齐次变换矩阵列表 (4x4)
        """
        if not os.path.exists(pose_file):
            raise FileNotFoundError(f"机器人位姿文件不存在: {pose_file}")
        
        poses = []
        line_count = 0
        
        with open(pose_file, 'r') as f:
            lines = f.readlines()
            
        print(f"正在读取机器人位姿文件: {pose_file}")
        
        for line_num, line in enumerate(lines):
            line = line.strip()
            
            # 跳过空行和注释行
            if not line or line.startswith('#'):
                continue
                
            # 分割数据
            data = line.split()
            
            if len(data) != 16:
                if len(data) > 0:  # 如果不是空行
                    print(f"警告: 第 {line_num+1} 行数据格式错误，期望16个数字，实际{len(data)}个，跳过")
                continue
            
            try:
                # 转换为浮点数
                values = list(map(float, data))
                
                # 重构为4x4矩阵
                T = np.array(values, dtype=np.float64).reshape(4, 4)
                
                # 验证变换矩阵的有效性
                if not np.allclose(T[3, :], [0, 0, 0, 1], atol=1e-6):
                    T[3, :] = [0, 0, 0, 1]
                
                # 检查旋转矩阵的正交性
                R_mat = T[:3, :3]
                det = np.linalg.det(R_mat)
                
                # 如果行列式不是1，进行正交化
                if abs(det - 1.0) > 0.01:
                    # 使用SVD进行正交化
                    U, _, Vt = np.linalg.svd(R_mat)
                    R_mat = U @ Vt
                    # 确保右手坐标系
                    if np.linalg.det(R_mat) < 0:
                        R_mat = U @ np.diag([1, 1, -1]) @ Vt
                    T[:3, :3] = R_mat
                
                poses.append(T)
                line_count += 1
                
                if line_count % 10 == 0:
                    print(f"  已加载 {line_count} 个位姿...")
                
            except Exception as e:
                print(f"警告: 处理第 {line_num+1} 行时发生错误: {e}")
                continue
        
        print(f"成功加载 {line_count} 个机器人位姿")
        
        if line_count == 0:
            raise ValueError("未找到有效的机器人位姿数据")
        
        return poses
    
    def detect_chessboard_corners(self, image_paths):
        """
        检测所有图像中的棋盘格角点
        
        返回:
            all_corners: 角点列表
            valid_indices: 有效图像的索引列表
            valid_images: 有效图像路径列表
        """
        if not image_paths:
            raise ValueError("图像路径列表为空")
        
        all_corners = []
        valid_indices = []
        valid_images = []
        
        print("\n开始检测棋盘格角点...")
        print("-" * 60)
        
        for idx, img_path in enumerate(image_paths):
            if not os.path.exists(img_path):
                print(f"警告: 图像文件不存在: {img_path}")
                continue
                
            img = cv2.imread(img_path)
            if img is None:
                print(f"警告: 无法读取图像: {img_path}")
                continue
            
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            
            # 查找棋盘格角点
            ret, corners = cv2.findChessboardCorners(
                gray, self.pattern_size,
                cv2.CALIB_CB_ADAPTIVE_THRESH + 
                cv2.CALIB_CB_NORMALIZE_IMAGE + 
                cv2.CALIB_CB_FAST_CHECK
            )
            
            if ret:
                # 亚像素精确化
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                
                all_corners.append(corners)
                valid_indices.append(idx)
                valid_images.append(img_path)
                
                print(f"✓ {os.path.basename(img_path):20s} | 找到 {len(corners)} 个角点")
                
                # 显示前几个检测结果
                if len(valid_indices) <= 3:
                    img_display = cv2.drawChessboardCorners(img.copy(), self.pattern_size, corners, ret)
                    display_name = f"Chessboard - {os.path.basename(img_path)}"
                    height, width = img.shape[:2]
                    display_size = (width // 2, height // 2)
                    img_resized = cv2.resize(img_display, display_size)
                    cv2.imshow(display_name, img_resized)
                    cv2.waitKey(500)
                    
            else:
                print(f"✗ {os.path.basename(img_path):20s} | 未检测到角点")
        
        cv2.destroyAllWindows()
        
        if all_corners:
            print(f"\n角点检测完成: {len(all_corners)}/{len(image_paths)} 张图像有效")
        else:
            print("\n错误: 未在任何图像中检测到角点!")
            raise ValueError("未检测到任何棋盘格角点")
            
        return all_corners, valid_indices, valid_images
    
    def calibrate_hand_eye(self, robot_poses, image_paths):
        """
        执行手眼标定 (Eye-in-Hand配置)
        
        返回:
            T_cam2gripper: 相机到机械臂末端的变换矩阵 (4x4)
            errors: 误差信息
        """
        # 1. 检测棋盘格角点
        print("\n" + "=" * 60)
        print("步骤1: 棋盘格角点检测")
        print("=" * 60)
        
        all_corners, valid_indices, valid_images = self.detect_chessboard_corners(image_paths)
        
        # 检查有效数据数量
        if len(all_corners) < 3:
            raise ValueError(f"需要至少3张有效图像进行标定，当前只有 {len(all_corners)} 张")
        
        # 2. 筛选对应的机器人位姿
        valid_robot_poses = [robot_poses[i] for i in valid_indices]
        
        print(f"\n使用 {len(valid_robot_poses)} 组有效数据进行标定")
        
        # 3. 计算相机到标定板的变换
        print("\n" + "=" * 60)
        print("步骤2: 计算相机到标定板的变换")
        print("=" * 60)
        
        camera_to_target_poses = []
        reprojection_errors = []
        
        print("使用solvePnP计算变换矩阵...")
        
        for idx, (corners, img_path) in enumerate(zip(all_corners, valid_images)):
            # 使用solvePnP计算相机到标定板的变换
            ret, rvec, tvec = cv2.solvePnP(
                self.objp, corners, 
                self.camera_matrix, self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE
            )
            
            if not ret:
                print(f"警告: 图像 {os.path.basename(img_path)} 的solvePnP失败")
                continue
            
            # 将旋转向量转换为旋转矩阵
            R_ct, _ = cv2.Rodrigues(rvec)
            
            # 创建齐次变换矩阵 (相机到标定板)
            T_ct = np.eye(4, dtype=np.float64)
            T_ct[:3, :3] = R_ct
            T_ct[:3, 3] = tvec.flatten()
            
            camera_to_target_poses.append(T_ct)
            
            # 计算重投影误差
            projected_points, _ = cv2.projectPoints(
                self.objp, rvec, tvec, 
                self.camera_matrix, self.dist_coeffs
            )
            error = cv2.norm(corners, projected_points, cv2.NORM_L2) / len(projected_points)
            reprojection_errors.append(error)
        
        if len(camera_to_target_poses) < 3:
            raise ValueError(f"有效solvePnP结果不足 {len(camera_to_target_poses)} < 3")
        
        # 重投影误差统计
        print(f"\n重投影误差统计:")
        print(f"  平均误差: {np.mean(reprojection_errors):.4f} 像素")
        print(f"  最大误差: {np.max(reprojection_errors):.4f} 像素")
        print(f"  最小误差: {np.min(reprojection_errors):.4f} 像素")
        
        # 4. 手眼标定
        print("\n" + "=" * 60)
        print("步骤3: 手眼标定")
        print("=" * 60)
        
        # 准备数据
        R_gripper2base = []
        t_gripper2base = []
        R_target2cam = []
        t_target2cam = []
        
        for i in range(len(valid_robot_poses)):
            # 机器人末端到基座的变换 (gripper -> base)
            T_gb = valid_robot_poses[i]
            R_gripper2base.append(T_gb[:3, :3])
            t_gripper2base.append(T_gb[:3, 3])
            
            # 标定板到相机的变换 (target -> camera)
            T_tc = camera_to_target_poses[i]
            R_target2cam.append(T_tc[:3, :3])
            t_target2cam.append(T_tc[:3, 3])
        
        # 尝试不同的标定方法
        methods = [
            ('TSAI', cv2.CALIB_HAND_EYE_TSAI),
            ('PARK', cv2.CALIB_HAND_EYE_PARK),
            ('HORAUD', cv2.CALIB_HAND_EYE_HORAUD),
            ('ANDREFF', cv2.CALIB_HAND_EYE_ANDREFF),
            ('DANIILIDIS', cv2.CALIB_HAND_EYE_DANIILIDIS)
        ]
        
        best_error = float('inf')
        best_T = None
        best_method_name = None
        best_errors = None
        
        print("尝试不同标定方法:")
        print("-" * 50)
        
        for method_name, method in methods:
            try:
                R_temp, t_temp = cv2.calibrateHandEye(
                    R_gripper2base, t_gripper2base,
                    R_target2cam, t_target2cam,
                    method=method
                )
                
                # 构建变换矩阵
                T_temp = np.eye(4, dtype=np.float64)
                T_temp[:3, :3] = R_temp
                T_temp[:3, 3] = t_temp.flatten()
                
                # 计算误差
                errors = self.calculate_hand_eye_error(
                    valid_robot_poses, camera_to_target_poses, T_temp
                )
                
                # 加权总误差
                weighted_error = errors['avg_rotation_error'] * 180/math.pi + errors['avg_translation_error'] * 1000
                
                print(f"{method_name:12s} | "
                      f"旋转误差: {errors['avg_rotation_error']*180/math.pi:6.3f}° | "
                      f"平移误差: {errors['avg_translation_error']*1000:6.3f}mm")
                
                if weighted_error < best_error:
                    best_error = weighted_error
                    best_T = T_temp
                    best_method_name = method_name
                    best_errors = errors
                    
            except Exception as e:
                print(f"{method_name:12s} | 失败: {str(e)}")
        
        if best_T is None:
            raise ValueError("所有手眼标定方法都失败")
        
        print(f"\n✓ 选择方法: {best_method_name}")
        
        return best_T, best_errors
    
    def calculate_hand_eye_error(self, robot_poses, camera_poses, T_cam2gripper):
        """
        计算手眼标定误差
        """
        rotation_errors = []
        translation_errors = []
        
        n = len(robot_poses)
        
        # 计算所有位姿对之间的误差
        for i in range(n):
            for j in range(i+1, n):
                # 机器人末端运动: A = inv(T_i) * T_j
                A = np.linalg.inv(robot_poses[i]) @ robot_poses[j]
                
                # 相机运动: B = T_i * inv(T_j)
                B = camera_poses[i] @ np.linalg.inv(camera_poses[j])
                
                # 计算左侧: A * X
                left_side = A @ T_cam2gripper
                
                # 计算右侧: X * B
                right_side = T_cam2gripper @ B
                
                # 计算旋转误差
                R_error = left_side[:3, :3] @ right_side[:3, :3].T
                angle = np.arccos(np.clip((np.trace(R_error) - 1) / 2, -1.0, 1.0))
                rotation_errors.append(abs(angle))
                
                # 计算平移误差
                t_error = np.linalg.norm(left_side[:3, 3] - right_side[:3, 3])
                translation_errors.append(t_error)
        
        # 计算统计信息
        errors = {
            'avg_rotation_error': np.mean(rotation_errors),
            'avg_translation_error': np.mean(translation_errors),
            'max_rotation_error': np.max(rotation_errors),
            'max_translation_error': np.max(translation_errors),
            'std_rotation_error': np.std(rotation_errors),
            'std_translation_error': np.std(translation_errors)
        }
        
        return errors
    
    def save_result(self, T_cam2gripper, output_file, errors=None):
        """
        保存结果到YAML文件
        """
        # 将矩阵转换为列表以便YAML序列化
        matrix_list = T_cam2gripper.tolist()
        
        # 提取四元数和欧拉角表示
        r = R.from_matrix(T_cam2gripper[:3, :3])
        quaternion = r.as_quat()  # [x, y, z, w]
        euler_angles_rad = r.as_euler('xyz')  # 以弧度为单位
        euler_angles_deg = r.as_euler('xyz', degrees=True)  # 以度为单位
        
        # 构建结果字典
        result = {
            'T_cam2gripper': {
                'matrix': matrix_list,
                'shape': [4, 4],
                'dtype': 'float64'
            },
            'translation': {
                'x': float(T_cam2gripper[0, 3]),
                'y': float(T_cam2gripper[1, 3]),
                'z': float(T_cam2gripper[2, 3]),
                'units': 'meters'
            },
            'rotation': {
                'quaternion': {
                    'x': float(quaternion[0]),
                    'y': float(quaternion[1]),
                    'z': float(quaternion[2]),
                    'w': float(quaternion[3]),
                    'note': '四元数顺序: [x, y, z, w]'
                },
                'euler_angles_rad': {
                    'x': float(euler_angles_rad[0]),
                    'y': float(euler_angles_rad[1]),
                    'z': float(euler_angles_rad[2]),
                    'sequence': 'XYZ',
                    'units': 'radians'
                },
                'euler_angles_deg': {
                    'x': float(euler_angles_deg[0]),
                    'y': float(euler_angles_deg[1]),
                    'z': float(euler_angles_deg[2]),
                    'sequence': 'XYZ',
                    'units': 'degrees'
                },
                'matrix': T_cam2gripper[:3, :3].tolist()
            },
            'calibration_info': {
                'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                'method': 'Tsai-Lenz (OpenCV)',
                'configuration': 'Eye-in-Hand (相机固定在机械臂末端)',
                'note': 'T_cam2gripper: 从相机坐标系到机械臂末端坐标系的变换'
            }
        }
        
        # 添加误差信息
        if errors:
            result['calibration_errors'] = {
                'rotation_error_deg': {
                    'average': float(errors['avg_rotation_error'] * 180 / math.pi),
                    'max': float(errors['max_rotation_error'] * 180 / math.pi),
                    'std': float(errors['std_rotation_error'] * 180 / math.pi),
                    'units': 'degrees'
                },
                'translation_error_mm': {
                    'average': float(errors['avg_translation_error'] * 1000),
                    'max': float(errors['max_translation_error'] * 1000),
                    'std': float(errors['std_translation_error'] * 1000),
                    'units': 'millimeters'
                }
            }
        
        # 写入文件
        with open(output_file, 'w') as f:
            yaml.dump(result, f, default_flow_style=False, indent=2)
        
        print(f"\n结果已保存到: {os.path.abspath(output_file)}")
        
        # 在控制台打印结果
        self.print_calibration_result(T_cam2gripper, quaternion, euler_angles_deg, errors)
    
    def print_calibration_result(self, T_cam2gripper, quaternion, euler_angles_deg, errors):
        """
        在控制台打印标定结果
        """
        print("\n" + "=" * 60)
        print("标定结果")
        print("=" * 60)
        
        print("\n相机到机械臂末端的变换矩阵 T_cam2gripper (4x4):")
        print("+" + "-" * 58 + "+")
        for i in range(4):
            print(f"| {T_cam2gripper[i, 0]:12.6f}  {T_cam2gripper[i, 1]:12.6f}  "
                  f"{T_cam2gripper[i, 2]:12.6f}  {T_cam2gripper[i, 3]:12.6f} |")
        print("+" + "-" * 58 + "+")
        
        print(f"\n平移向量:")
        print(f"  X: {T_cam2gripper[0, 3]:.6f} m  ({T_cam2gripper[0, 3]*1000:.3f} mm)")
        print(f"  Y: {T_cam2gripper[1, 3]:.6f} m  ({T_cam2gripper[1, 3]*1000:.3f} mm)")
        print(f"  Z: {T_cam2gripper[2, 3]:.6f} m  ({T_cam2gripper[2, 3]*1000:.3f} mm)")
        
        print(f"\n四元数 (w, x, y, z):")
        print(f"  w: {quaternion[3]:.6f}")
        print(f"  x: {quaternion[0]:.6f}")
        print(f"  y: {quaternion[1]:.6f}")
        print(f"  z: {quaternion[2]:.6f}")
        
        print(f"\n欧拉角 (XYZ顺序):")
        print(f"  X轴: {euler_angles_deg[0]:.2f}°")
        print(f"  Y轴: {euler_angles_deg[1]:.2f}°")
        print(f"  Z轴: {euler_angles_deg[2]:.2f}°")
        
        # 验证旋转矩阵
        print(f"\n旋转矩阵验证:")
        R_mat = T_cam2gripper[:3, :3]
        det = np.linalg.det(R_mat)
        orthogonality = np.max(np.abs(R_mat @ R_mat.T - np.eye(3)))
        
        print(f"  行列式: {det:.6f} {'✓' if abs(det - 1.0) < 0.01 else '✗'}")
        print(f"  正交性: {orthogonality:.6e} {'✓' if orthogonality < 1e-6 else '✗'}")
        
        if errors:
            print(f"\n标定误差:")
            print(f"  平均旋转误差: {errors['avg_rotation_error']*180/math.pi:.4f}°")
            print(f"  最大旋转误差: {errors['max_rotation_error']*180/math.pi:.4f}°")
            print(f"  平均平移误差: {errors['avg_translation_error']*1000:.4f} mm")
            print(f"  最大平移误差: {errors['max_translation_error']*1000:.4f} mm")
        
        print("\n使用说明:")
        print("  1. T_cam2gripper 将相机坐标系中的点转换到机械臂末端坐标系")
        print("  2. 点转换: P_gripper = T_cam2gripper @ P_camera")
        print("  3. 相机->世界: P_world = T_base2gripper @ T_cam2gripper @ P_camera")


def find_images(image_folder, extensions=None):
    """
    在指定文件夹中查找图像文件
    
    参数:
        image_folder: 图像文件夹路径
        extensions: 图像扩展名列表
        
    返回:
        排序后的图像路径列表
    """
    if extensions is None:
        extensions = ['*.jpg', '*.jpeg', '*.png', '*.bmp', '*.tiff', '*.tif']
    
    image_paths = []
    for ext in extensions:
        pattern = os.path.join(image_folder, '**', ext) if '*' in ext else os.path.join(image_folder, ext)
        image_paths.extend(glob.glob(pattern, recursive=True))
    
    # 按文件名排序
    image_paths.sort(key=lambda x: os.path.basename(x))
    
    return image_paths


def main():
    """
    主函数 - 独立版本，硬编码相机参数
    """
    print("\n" + "=" * 70)
    print("手眼标定程序 (Eye-in-Hand) - 独立版本")
    print("相机到机械臂末端的变换矩阵标定")
    print("=" * 70)
    
    # ============================ 硬编码相机参数 ============================
    # 根据您提供的相机内参信息
    print("\n[1/5] 设置相机内参...")
    
    # 相机内参矩阵
    fx = 775.952503488026991   # 焦距 fx
    fy = 775.952503488026991   # 焦距 fy
    cx = 825.154450261780084   # 主点 cx
    cy = 601.070680628272271   # 主点 cy
    
    camera_matrix = np.array([
        [fx, 0, cx],
        [0, fy, cy],
        [0, 0, 1]
    ], dtype=np.float64)
    
    # 畸变系数 (假设无畸变)
    dist_coeffs = np.array([0.00000, 0.00000, 0.00000, 0.00000, 0.00000], dtype=np.float64)
    
    # 棋盘格参数
    pattern_size = (19, 19)    # 20x20棋盘格有19x19内部角点
    square_size = 0.01         # 每个格子0.01米
    
    print(f"相机内参:")
    print(f"  焦距: fx={fx:.2f}, fy={fy:.2f}")
    print(f"  主点: cx={cx:.2f}, cy={cy:.2f}")
    print(f"  畸变系数: {dist_coeffs}")
    print(f"  棋盘格内部角点: {pattern_size}")
    print(f"  方格大小: {square_size} m")
    
    # ============================ 文件路径配置 ============================
    # 注意：这些文件需要在当前目录下
    robot_pose_file = "/home/zzz/ros2_ws/src/my_ur10e_control/camera/robot_pos.txt"    # 机器人位姿文件
    image_folder = "/home/zzz/ros2_ws/src/my_ur10e_control/camera/rc_imgs"             # 图像文件夹
    output_file = "/home/zzz/ros2_ws/src/my_ur10e_control/camera/calib_output/T_cam2gripper.yaml"   # 输出文件
    
    # 2. 初始化标定器
    calibrator = HandEyeCalibration(
        camera_matrix, dist_coeffs, pattern_size, square_size
    )
    
    # 3. 加载机器人位姿
    print(f"\n[2/5] 加载机器人位姿...")
    if not os.path.exists(robot_pose_file):
        print(f"错误: 机器人位姿文件不存在: {robot_pose_file}")
        print(f"请确保 {robot_pose_file} 在当前目录下")
        sys.exit(1)
    
    try:
        robot_poses = calibrator.load_robot_poses(robot_pose_file)
    except Exception as e:
        print(f"错误: 无法加载机器人位姿: {e}")
        sys.exit(1)
    
    # 4. 获取图像路径
    print(f"\n[3/5] 查找图像文件...")
    if not os.path.exists(image_folder):
        print(f"错误: 图像文件夹不存在: {image_folder}")
        print(f"请确保 {image_folder} 文件夹在当前目录下")
        sys.exit(1)
    
    image_paths = find_images(image_folder)
    
    if not image_paths:
        print(f"错误: 在 {image_folder} 中未找到图像文件")
        sys.exit(1)
    
    print(f"找到 {len(image_paths)} 张图像")
    
    # 5. 检查数据一致性
    if len(image_paths) != len(robot_poses):
        print(f"警告: 图像数量({len(image_paths)})与位姿数量({len(robot_poses)})不匹配")
        print("将使用两者中较小的数量")
        min_len = min(len(image_paths), len(robot_poses))
        robot_poses = robot_poses[:min_len]
        image_paths = image_paths[:min_len]
        print(f"使用 {min_len} 组数据进行标定")
    
    # 6. 执行手眼标定
    print(f"\n[4/5] 执行手眼标定...")
    try:
        T_cam2gripper, errors = calibrator.calibrate_hand_eye(robot_poses, image_paths)
    except Exception as e:
        print(f"错误: 手眼标定失败: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    
    # 7. 保存结果
    print(f"\n[5/5] 保存标定结果...")
    try:
        calibrator.save_result(T_cam2gripper, output_file, errors)
    except Exception as e:
        print(f"错误: 无法保存结果: {e}")
        sys.exit(1)
    
    print("\n" + "=" * 70)
    print("标定完成!")
    print(f"结果保存在: {os.path.abspath(output_file)}")
    print("=" * 70)
    
    # 8. 验证结果
    print(f"\n验证标定结果:")
    
    # 验证变换矩阵的有效性
    print("1. 变换矩阵验证:")
    print(f"   行列式接近1: {abs(np.linalg.det(T_cam2gripper[:3, :3]) - 1.0) < 0.01}")
    print(f"   正交性检查: {np.allclose(T_cam2gripper[:3, :3] @ T_cam2gripper[:3, :3].T, np.eye(3), atol=1e-6)}")
    
    # 提供使用示例
    print(f"\n2. 使用示例:")
    print("   将相机坐标系中的点转换到机械臂末端坐标系:")
    print("   point_in_gripper = T_cam2gripper @ point_in_camera")
    
    print(f"\n3. 在Python中加载结果:")
    print(f'   import yaml')
    print(f'   import numpy as np')
    print(f'   with open("{output_file}", "r") as f:')
    print('       data = yaml.safe_load(f)')
    print('   T_cam2gripper = np.array(data["T_cam2gripper"]["matrix"])')
    
    print(f"\n标定完成！")


if __name__ == "__main__":
    main()