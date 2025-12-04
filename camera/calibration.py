import numpy as np
import cv2
import glob
import yaml

class CameraRobotCalibration:
    def __init__(self, config, robot_poses):
        """
        初始化标定类，设置棋盘格尺寸、相机内参、畸变系数以及机器人末端位置
        
        :param config: (dict) 从配置文件读取的参数字典
        :param robot_poses: (list) 机器人末端的位姿列表（4x4变换矩阵）
        """
        self.chessboard_size = tuple(config['chessboard']['size'])  # 棋盘格尺寸（行，列）
        self.square_size = config['chessboard']['square_size']  # 格子边长（单位：米）
        
        # 相机内参
        camera_matrix = config['camera']['matrix']
        self.camera_matrix = np.array([
            [camera_matrix['fx'], 0, camera_matrix['cx']],
            [0, camera_matrix['fy'], camera_matrix['cy']],
            [0, 0, 1]
        ], dtype=np.float32)
        
        self.dist_coeffs = np.array(config['camera']['distortion_coeffs'], dtype=np.float32)  # 畸变系数
        
        # 机器人末端位置（4x4变换矩阵）
        self.robot_end_poses = [np.array(pose).reshape(4, 4) for pose in robot_poses]

        self.object_points_list = self._generate_object_points()

    def _generate_object_points(self):
        """
        生成标定板的物理世界坐标系角点
        
        :return: (ndarray) 世界坐标系中的标定板角点
        """
        object_points = np.zeros((np.prod(self.chessboard_size), 3), dtype=np.float32)
        object_points[:, :2] = np.indices(self.chessboard_size).T.reshape(-1, 2)
        object_points *= self.square_size
        return object_points

    def calibrate_camera(self, image_paths):
        """
        通过标定图像进行相机标定，提取角点
        
        :param image_paths: (list) 标定图像路径列表
        :return: (list) 图像中的角点，(list) 物体坐标
        """
        image_points = []
        object_points_list = []

        for image_path in image_paths:
            img = cv2.imread(image_path)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # 查找棋盘格角点
            ret, corners = cv2.findChessboardCorners(gray, self.chessboard_size, None)

            if ret:
                cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                 criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1))

                image_points.append(corners)
                object_points_list.append(self.object_points_list)

        return image_points, object_points_list

    def calculate_camera_to_robot_transform(self, image_points, object_points_list):
        """
        使用已知的角点和机器人末端位置标定相机与机器人坐标系的转换关系
        
        :param image_points: (list) 角点列表
        :param object_points_list: (list) 物体坐标列表
        :return: (ndarray) 旋转矩阵 R 和 平移向量 T
        """
        # 使用 solvePnP 计算外参（旋转矩阵和平移向量）
        R_list = []
        T_list = []

        for i in range(len(image_points)):
            # 使用相机内参和畸变系数来计算外参
            ret, rvec, tvec = cv2.solvePnP(object_points_list[i], image_points[i], self.camera_matrix, self.dist_coeffs)

            # 将旋转向量转换为旋转矩阵
            R, _ = cv2.Rodrigues(rvec)

            # 储存外参
            R_list.append(R)
            T_list.append(tvec)

        # 计算外参的平均值（可以选择其他方法，比如最小二乘法拟合）
        R_avg = np.mean(R_list, axis=0)
        T_avg = np.mean(T_list, axis=0)

        return R_avg, T_avg

    def calculate_transform_from_robot_to_base(self, R_avg, T_avg):
        """
        计算相机坐标系到机器人基坐标系的转换矩阵
        
        :param R_avg: (ndarray) 平均旋转矩阵
        :param T_avg: (ndarray) 平均平移向量
        :return: (ndarray) 机器人坐标系到基坐标系的转换矩阵
        """
        # 假设机器人末端到基坐标系的转换为单位矩阵和零向量（如果有实际数据，可以替换）
        R_b = np.eye(3)
        T_b = np.array([0, 0, 0])

        # 计算相机到机器人基坐标系的转换
        R_base = np.dot(R_b, R_avg)
        T_base = T_b - np.dot(R_b, T_avg)

        return R_base, T_base

# 从配置文件读取参数
def load_config(config_path):
    """
    从 YAML 配置文件中加载参数
    
    :param config_path: (str) 配置文件路径
    :return: (dict) 配置字典
    """
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)
    return config

# 从 robot.txt 文件读取机器人位姿
def load_robot_poses(robot_poses_path):
    """
    从 robot.txt 文件中读取机器人末端位姿数据
    
    :param robot_poses_path: (str) 机器人位姿文件路径
    :return: (list) 机器人末端位姿变换矩阵列表
    """
    robot_poses = []
    with open(robot_poses_path, 'r') as file:
        for line in file.readlines():
            # 每行是16个浮点数，代表一个4x4的变换矩阵
            values = list(map(float, line.strip().split()))
            if len(values) == 16:
                robot_poses.append(values)
    return robot_poses

# 主函数示例
def main():
    # 从 YAML 配置文件读取参数
    config_path = "config.yaml"
    config = load_config(config_path)

    # 从 robot.txt 文件读取机器人位姿
    robot_poses_path = "robot.txt"
    robot_poses = load_robot_poses(robot_poses_path)

    # 创建标定对象
    calibration = CameraRobotCalibration(config, robot_poses)

    # 读取标定图像路径
    image_paths = [f"imgs/{i}.jpg" for i in range(1, 56)]

    # 相机标定，获取角点
    image_points, object_points_list = calibration.calibrate_camera(image_paths)

    # 计算相机到机器人末端的外参（旋转矩阵和平移向量）
    R_avg, T_avg = calibration.calculate_camera_to_robot_transform(image_points, object_points_list)

    # 计算相机到机器人基坐标系的转换矩阵
    R_base, T_base = calibration.calculate_transform_from_robot_to_base(R_avg, T_avg)

    # 输出最终结果
    print("Camera to Robot Base Transformation:")
    print("Rotation Matrix R:\n", R_base)
    print("Translation Vector T:\n", T_base)

if __name__ == "__main__":
    main()
