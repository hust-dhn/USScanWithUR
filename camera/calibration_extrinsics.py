import numpy as np
import cv2
import glob
import yaml
import os
from typing import List, Tuple


# ===== User-configurable defaults (modify paths here if files move) =====
BOOL_LC = False  # True for left camera, False for right camera

# LEFT
if BOOL_LC:
    DEFAULT_CONFIG_PATH = 'camera/config_lc.yaml'
    DEFAULT_IMAGE_FOLDER = 'camera/lc_imgs/'
    DEFAULT_OUTPUT_DIR = 'camera/calib_output/'
    DEFAULT_OUTPUT_INTRINSICS_FILE = 'intrinsics_lc.yaml'
    DEFAULT_ROBOT_POSES_FILE = 'camera/robot_pos.txt'
# RIGHT
else:
    DEFAULT_CONFIG_PATH = 'camera/config_rc.yaml'
    DEFAULT_IMAGE_FOLDER = 'camera/rc_imgs/'
    DEFAULT_OUTPUT_DIR = 'camera/calib_output/'
    DEFAULT_OUTPUT_INTRINSICS_FILE = 'intrinsics_rc.yaml'
    DEFAULT_ROBOT_POSES_FILE = 'camera/robot_pos.txt'
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


def create_aruco_board(board_config: dict):
    """创建ArUco板对象"""
    dictionary_name = board_config['dictionary']
    marker_length = board_config['marker_length']
    board_size = board_config['board_size']
    square_size = board_config['square_size']
    
    # 获取字典
    aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
    
    # 创建板子
    board = cv2.aruco.GridBoard((board_size[1], board_size[0]), marker_length, square_size, aruco_dict)
    return board, aruco_dict


def detect_aruco_board(image_folder: str, board_config: dict):
    """
    在指定文件夹中检测ArUco板。
    返回：used_paths, R_target2cam_list, t_target2cam_list, object_points_list, image_points_list
    """
    board, aruco_dict = create_aruco_board(board_config)
    
    # 列出所有文件并按文件名中的数字索引排序
    all_paths = glob.glob(os.path.join(image_folder, '*'))
    def extract_index(p):
        name = os.path.basename(p)
        import re
        m = re.search(r"(\d+)", name)
        if m:
            return int(m.group(1))
        return 10**9
    image_paths = sorted(all_paths, key=extract_index)
    
    used_paths = []
    R_target2cam_list = []
    t_target2cam_list = []
    object_points_list = []
    image_points_list = []
    
    for p in image_paths:
        img = cv2.imread(p)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # 检测markers
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict)
        if ids is not None and len(ids) > 0:
            # 估计板子姿态
            ret, rvec, tvec = cv2.aruco.estimatePoseBoard(corners, ids, board, 
                                                         np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=np.float64), 
                                                         np.zeros(5), None, None)
            if ret > 0:
                R, _ = cv2.Rodrigues(rvec)
                R_target2cam_list.append(R)
                t_target2cam_list.append(tvec.reshape(3))
                used_paths.append(p)
                
                # 为内参标定收集object points和image points
                obj_points, img_points = board.matchImagePoints(corners, ids)
                if obj_points is not None and img_points is not None:
                    object_points_list.append(obj_points)
                    image_points_list.append(img_points)
            else:
                print(f"Warning: ArUco board pose estimation failed in {p}")
        else:
            print(f"Warning: No ArUco markers found in {p}")
    
    return used_paths, R_target2cam_list, t_target2cam_list, object_points_list, image_points_list


def visualize_aruco_detection(image_folder: str, used_paths: List[str], board_config: dict, output_folder: str):
    """
    可视化ArUco板检测结果，在检测到的图片上绘制markers，保存标注图到输出文件夹
    """
    board, aruco_dict = create_aruco_board(board_config)
    
    os.makedirs(output_folder, exist_ok=True)
    print(f'[Info] 保存ArUco检测结果到 {output_folder}...')
    
    for p in used_paths:
        img = cv2.imread(p)
        if img is None:
            continue
        
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict)
        
        # 绘制检测到的markers
        vis = img.copy()
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(vis, corners, ids)
            
            # 如果能估计姿态，也绘制轴
            ret, rvec, tvec = cv2.aruco.estimatePoseBoard(corners, ids, board, 
                                                         np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=np.float64), 
                                                         np.zeros(5), None, None)
            if ret > 0:
                cv2.drawFrameAxes(vis, np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=np.float64), np.zeros(5), rvec, tvec, 0.05)
        
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


def main(config_path='config_lc.yaml', image_folder='lc_imgs', robot_poses_file='robot_pos.txt'):
    # 读取配置
    config = load_config(config_path)
    board_config = config['calibration_board']
    board_type = board_config['type']
    
    if board_type != 'aruco':
        raise ValueError(f"Unsupported board type: {board_type}. Only 'aruco' is supported.")

    # 检测ArUco板
    print(f"Detecting ArUco board in {image_folder}...")
    used_paths, R_target2cam_list, t_target2cam_list, object_points_list, image_points_list = detect_aruco_board(image_folder, board_config)
    if len(R_target2cam_list) == 0:
        print('No ArUco boards found. Abort')
        return
    
    # 可视化检测结果
    visualize_aruco_detection(image_folder, used_paths, board_config, DEFAULT_OUTPUT_DIR)

    # 尝试从配置读取内参，如果没有或为占位值则标定内参
    cam_cfg = config.get('camera', {})
    have_intrinsics = False
    try:
        fx = cam_cfg['matrix']['fx']
        fy = cam_cfg['matrix']['fy']
        cx = cam_cfg['matrix']['cx']
        cy = cam_cfg['matrix']['cy']
        camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
        dist_coeffs = np.array(cam_cfg.get('distortion_coeffs', [0, 0, 0, 0, 0]), dtype=np.float64)
        have_intrinsics = True
        print('[Info] 使用配置文件中的相机内参')
    except Exception:
        have_intrinsics = False

    # 如果没有内参，执行相机标定
    if not have_intrinsics and len(object_points_list) > 0:
        sample_img = cv2.imread(used_paths[0])
        image_size = (sample_img.shape[1], sample_img.shape[0])
        print('[Info] 标定相机内参...')
        camera_matrix, dist_coeffs, _, _ = calibrate_intrinsics(object_points_list, image_points_list, image_size)
        print('Estimated camera matrix:')
        print(camera_matrix)
        print('Estimated dist coeffs:')
        print(dist_coeffs)
    elif not have_intrinsics:
        print('[Warning] No intrinsics available and no valid detections for calibration.')

    # 读取机器人位姿
    robot_poses = load_robot_poses(robot_poses_file)
    if len(robot_poses) == 0:
        print('No robot poses found. Abort')
        return

    # 对齐样本数量
    n = min(len(robot_poses), len(R_target2cam_list))
    robot_poses = robot_poses[:n]
    R_target2cam_list = R_target2cam_list[:n]
    t_target2cam_list = t_target2cam_list[:n]

    # 执行手眼标定
    print('[Info] 执行手眼标定 (calibrateHandEye)')
    T_cam2gripper = hand_eye_calibration(robot_poses, R_target2cam_list, t_target2cam_list)

    print('\nResult: T_cam2gripper (camera -> gripper / TCP)')
    print(T_cam2gripper)

    # 保存结果
    out_dir = DEFAULT_OUTPUT_DIR
    os.makedirs(out_dir, exist_ok=True)
    save_transform_yaml(os.path.join(out_dir, 'T_cam2gripper.yaml'), T_cam2gripper)
    print(f'[Info] 结果已保存到 {out_dir}')
    print(f'[Info] 检测到 {len(used_paths)} 张有效的ArUco板图片')


if __name__ == '__main__':
    # 默认使用顶部常量（便于修改路径）
    main(config_path=DEFAULT_CONFIG_PATH, image_folder=DEFAULT_IMAGE_FOLDER, robot_poses_file=DEFAULT_ROBOT_POSES_FILE)
