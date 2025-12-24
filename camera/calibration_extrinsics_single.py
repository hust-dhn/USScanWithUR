import numpy as np
import cv2
import glob
import yaml
import os
from typing import List, Tuple


# ===== 单相机配置 =====
DEFAULT_CONFIG_PATH = 'camera/cfg/config_lc.yaml'  # 只使用左相机配置
DEFAULT_IMAGE_FOLDER = 'camera/imgs/'          # 单相机图像文件夹
DEFAULT_OUTPUT_DIR = 'camera/calib_output/'
DEFAULT_OUTPUT_INTRINSICS_FILE = 'camera/cfg/intrinsics.yaml'  # 单相机内参文件
DEFAULT_ROBOT_POSES_FILE = 'camera/cfg/robot_pos.txt'
# ======================================================================


def load_config(config_path: str) -> dict:
    """读取 YAML 配置文件"""
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


def load_robot_poses(robot_poses_path: str) -> List[np.ndarray]:
    """
    从位姿文件读取机器人位姿（每行16个浮点数，展开的4x4矩阵）
    返回：list of 4x4 numpy arrays
    """
    poses = []
    with open(robot_poses_path, 'r') as f:
        for line in f:
            vals = list(map(float, line.strip().split()))
            if len(vals) != 16:
                continue
            poses.append(np.array(vals).reshape(4, 4))
    return poses


def generate_object_points(chessboard_size: Tuple[int, int], square_size: float) -> np.ndarray:
    """生成单张棋盘格的物理角点（3D）"""
    objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
    objp *= square_size
    return objp


def detect_chessboards(image_folder: str, chessboard_size: Tuple[int, int], square_size: float):
    """
    在指定文件夹中检测棋盘角点。
    返回：image_paths_used, image_points(list of Nx1x2), object_points(list of NxM)
    """
    # 列出所有文件并按文件名中的数字索引排序，保证与拍照时的数值索引一一对应
    all_paths = glob.glob(os.path.join(image_folder, '*'))
    def extract_index(p):
        name = os.path.basename(p)
        # 提取文件名中的连续数字作为索引（例如 '12.jpg' -> 12），找不到则返回 large number
        import re
        m = re.search(r"(\d+)", name)
        if m:
            return int(m.group(1))
        return 10**9
    image_paths = sorted(all_paths, key=extract_index)
    image_points = []
    object_points = []
    used_paths = []

    objp = generate_object_points(chessboard_size, square_size)

    for p in image_paths:
        img = cv2.imread(p)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
        if ret:
            term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), term)
            image_points.append(corners)
            object_points.append(objp)
            used_paths.append(p)
        else:
            print(f"Warning: chessboard not found in {p}")

    return used_paths, image_points, object_points


def visualize_chessboard_detection(image_folder: str, used_paths: List[str], image_points: List[np.ndarray],
                                   chessboard_size: Tuple[int, int], output_folder: str):
    """
    可视化棋盘格检测结果，在检测到的图片上绘制角点，保存标注图到输出文件夹
    """
    os.makedirs(output_folder, exist_ok=True)
    print(f'[Info] 保存棋盘格检测结果到 {output_folder}...')
    
    for i, (p, corners) in enumerate(zip(used_paths, image_points)):
        img = cv2.imread(p)
        if img is None:
            continue
        
        # 在图上绘制检测到的棋盘角点
        vis = img.copy()
        cv2.drawChessboardCorners(vis, chessboard_size, corners, True)
        
        # 保存标注图
        fname = os.path.basename(p)
        out_path = os.path.join(output_folder, f'detected_{fname}')
        cv2.imwrite(out_path, vis)
    
    print(f'[Info] 保存完成：{len(used_paths)} 张图片已标注并保存')


def calibrate_intrinsics(object_points: List[np.ndarray], image_points: List[np.ndarray], image_size: Tuple[int, int]):
    """
    使用 cv2.calibrateCamera 估计相机内参和畸变。如果已有内参可跳过。
    返回 (camera_matrix, dist_coeffs, rvecs, tvecs)
    """
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(object_points, image_points, image_size, None, None)
    if not ret:
        raise RuntimeError('calibrateCamera failed')
    return camera_matrix, dist_coeffs, rvecs, tvecs


def compute_extrinsics_per_image(object_points: List[np.ndarray], image_points: List[np.ndarray],
                                  camera_matrix: np.ndarray, dist_coeffs: np.ndarray):
    """
    对每张图像使用 solvePnP 计算 target->cam 的外参（R,t），返回列表的 (R_mat, tvec)
    """
    R_list = []
    t_list = []
    for objp, imgp in zip(object_points, image_points):
        success, rvec, tvec = cv2.solvePnP(objp, imgp, camera_matrix, dist_coeffs)
        if not success:
            raise RuntimeError('solvePnP failed for one image')
        R, _ = cv2.Rodrigues(rvec)
        R_list.append(R)
        t_list.append(tvec.reshape(3))
    return R_list, t_list


def hand_eye_calibration(robot_poses: List[np.ndarray], R_target2cam: List[np.ndarray], t_target2cam: List[np.ndarray]):
    """
    使用 OpenCV 的 calibrateHandEye 求解手眼标定。
    输入：robot_poses: list of 4x4 base->gripper 矩阵（对应于每张图片的顺序）
          R_target2cam, t_target2cam: 列表，对应每张图片
    返回：T_cam2gripper (4x4), T_base2cam_ref (4x4 使用第一个位姿作为参考)
    """
    if len(robot_poses) != len(R_target2cam):
        n = min(len(robot_poses), len(R_target2cam))
        print(f"[Warning] robot poses ({len(robot_poses)}) and detected images ({len(R_target2cam)}) mismatch, using first {n} pairs")
        robot_poses = robot_poses[:n]
        R_target2cam = R_target2cam[:n]
        t_target2cam = t_target2cam[:n]

    # 构造 gripper->base 的旋转矩阵和位移（注意：robot_poses 是 base->gripper）
    R_gripper2base = []
    t_gripper2base = []
    for T in robot_poses:
        R = T[0:3, 0:3]
        t = T[0:3, 3]
        R_gripper2base.append(R)
        t_gripper2base.append(t)

    # OpenCV 的 calibrateHandEye 接受 lists of rotation matrices/translations
    try:
        R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(R_gripper2base, t_gripper2base,
                                                           R_target2cam, t_target2cam,
                                                           method=cv2.CALIB_HAND_EYE_TSAI)
    except Exception as e:
        raise RuntimeError(f"calibrateHandEye failed: {e}")

    # 构造 4x4 矩阵 X_cam2gripper（相机在末端 TCP / gripper 坐标系下的变换）
    T_cam2gripper = np.eye(4)
    T_cam2gripper[0:3, 0:3] = R_cam2gripper
    T_cam2gripper[0:3, 3] = t_cam2gripper.reshape(3)

    # 返回相机到末端（TCP）变换矩阵
    return T_cam2gripper


def save_transform_yaml(path: str, T: np.ndarray):
    """将 4x4 变换矩阵保存为 YAML 文件（平铺形式）"""
    data = {
        'transform': T.flatten().tolist()
    }
    with open(path, 'w') as f:
        yaml.safe_dump(data, f)


def save_intrinsics_yaml(path: str, camera_matrix: np.ndarray, dist_coeffs: np.ndarray, image_size: Tuple[int, int]):
    """将相机内参保存为 YAML 文件"""
    data = {
        'camera_matrix': camera_matrix.flatten().tolist(),
        'distortion_coefficients': dist_coeffs.flatten().tolist(),
        'image_size': list(image_size)
    }
    with open(path, 'w') as f:
        yaml.safe_dump(data, f)


def main(config_path='camera/config_lc.yaml', image_folder='camera/imgs', robot_poses_file='camera/robot_pos.txt'):
    # 读取配置
    config = load_config(config_path)
    chessboard = config['chessboard']
    cb_size = tuple(chessboard['size'])
    square_size = chessboard['square_size']

    # 检测棋盘角点
    print(f"[Info] 在 {image_folder} 中检测棋盘格...")
    used_paths, image_points, object_points = detect_chessboards(image_folder, cb_size, square_size)
    if len(image_points) == 0:
        print('[Error] 未检测到棋盘格，程序终止')
        return
    
    # 可视化检测结果
    visualize_chessboard_detection(image_folder, used_paths, image_points, cb_size, DEFAULT_OUTPUT_DIR)

    # 尝试从配置读取内参，如果没有或为占位值则标定内参
    cam_cfg = config.get('camera', {})
    have_intrinsics = False
    try:
        # 假设配置文件中包含相机内参
        fx = cam_cfg['sensor2']['matrix']['fx']
        fy = cam_cfg['sensor2']['matrix']['fy']
        cx = cam_cfg['sensor2']['matrix']['cx']
        cy = cam_cfg['sensor2']['matrix']['cy']
        camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
        print(f"camera_matrix: {camera_matrix}")
        dist_coeffs = np.array(cam_cfg['sensor2']['distortion_coeffs'], dtype=np.float64)
        print(f"dist_coeffs: {dist_coeffs}")
        have_intrinsics = True
        print('[Info] 使用配置文件中的相机内参')
    except Exception:
        have_intrinsics = False

    # 如果没有内参，执行相机标定
    sample_img = cv2.imread(used_paths[0])
    image_size = (sample_img.shape[1], sample_img.shape[0])
    if not have_intrinsics:
        print('[Info] 标定相机内参...')
        camera_matrix, dist_coeffs, _, _ = calibrate_intrinsics(object_points, image_points, image_size)
        print('[Info] 相机内参矩阵:')
        print(camera_matrix)
        print('[Info] 畸变系数:')
        print(dist_coeffs)
        
        # 保存内参到文件
        save_intrinsics_yaml(
            DEFAULT_OUTPUT_INTRINSICS_FILE,
            camera_matrix, dist_coeffs, image_size
        )
        print(f'[Info] 相机内参已保存到 {DEFAULT_OUTPUT_INTRINSICS_FILE}')

    # 计算每张图片的外参（target -> cam）
    print('[Info] 计算每张图片的外参（target->cam）')
    R_target2cam, t_target2cam = compute_extrinsics_per_image(object_points, image_points, camera_matrix, dist_coeffs)

    # 读取机器人位姿
    robot_poses = load_robot_poses(robot_poses_file)
    if len(robot_poses) == 0:
        print('[Error] 未找到机器人位姿文件，程序终止')
        return

    # 对齐样本数量
    n = min(len(robot_poses), len(R_target2cam))
    robot_poses = robot_poses[:n]
    R_target2cam = R_target2cam[:n]
    t_target2cam = t_target2cam[:n]
    
    print(f'[Info] 使用 {n} 个匹配的位姿-图像对进行手眼标定')

    # 执行手眼标定
    print('[Info] 执行手眼标定 (calibrateHandEye)')
    T_cam2gripper = hand_eye_calibration(robot_poses, R_target2cam, t_target2cam)

    print('\n[Result] T_cam2gripper (相机 -> 机器人末端 / TCP)')
    print(T_cam2gripper)

    # 保存结果
    out_dir = DEFAULT_OUTPUT_DIR
    os.makedirs(out_dir, exist_ok=True)
    save_transform_yaml(os.path.join(out_dir, 'T_cam2gripper.yaml'), T_cam2gripper)
    print(f'[Info] 手眼标定结果已保存到 {out_dir}/T_cam2gripper.yaml')
    print(f'[Info] 检测到 {len(used_paths)} 张有效的棋盘格图片')


if __name__ == '__main__':
    # 使用单相机配置
    print("=" * 60)
    print("单相机手眼标定系统")
    print("=" * 60)
    print(f"[Config] 配置文件: {DEFAULT_CONFIG_PATH}")
    print(f"[Config] 图像文件夹: {DEFAULT_IMAGE_FOLDER}")
    print(f"[Config] 机器人位姿文件: {DEFAULT_ROBOT_POSES_FILE}")
    print(f"[Config] 输出目录: {DEFAULT_OUTPUT_DIR}")
    print("=" * 60)
    
    main(
        config_path=DEFAULT_CONFIG_PATH, 
        image_folder=DEFAULT_IMAGE_FOLDER, 
        robot_poses_file=DEFAULT_ROBOT_POSES_FILE
    )