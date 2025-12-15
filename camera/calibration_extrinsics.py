import numpy as np
import cv2
import glob
import yaml
import os
from typing import List, Tuple, Optional


# ===== User-configurable defaults (modify paths here if files move) =====
BOOL_LC = True  # True for left camera, False for right camera

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
            poses.append(np.array(vals, dtype=np.float64).reshape(4, 4))
    return poses


def extract_index_from_path(p: str) -> int:
    """从文件名中提取数字索引，用于对齐机器人位姿"""
    name = os.path.basename(p)
    import re
    m = re.search(r"(\d+)", name)
    if m:
        return int(m.group(1))
    return 10**9

def get_squares_xy(board_size):
    # 约定：board_size: [squaresX, squaresY]  (列, 行)
    squaresX = int(board_size[0])
    squaresY = int(board_size[1])
    return squaresX, squaresY

def id_to_square_rc(marker_id: int, squaresX: int, squaresY: int):
    """
    OpenCV 生成的 ChArUco 默认 ID 排布：按行从上到下、列从左到右扫描，
    只在 (r+c)%2==0 的方块上放 marker，并从 0 开始依次编号。
    这个规则与你这张板（0~15）是一致的。
    """
    count = 0
    for r in range(squaresY):
        for c in range(squaresX):
            if (r + c) % 2 == 0:  # marker 所在方块
                if count == marker_id:
                    return r, c
                count += 1
    return None


def inv_T(T: np.ndarray) -> np.ndarray:
    """4x4 齐次变换求逆"""
    R = T[:3, :3]
    t = T[:3, 3:4]
    Ti = np.eye(4, dtype=np.float64)
    Ti[:3, :3] = R.T
    Ti[:3, 3:4] = -R.T @ t
    return Ti


def create_charuco_board(board_config: dict):
    """
    用 YAML 里的配置创建 ChArUco 板（你发的那种板是 ChArUco，不是纯 GridBoard）
    约定：board_size = [rows, cols]，即 [squaresY, squaresX]
    """
    dictionary_name = board_config['dictionary']          # e.g. "DICT_4X4_50"
    marker_length = float(board_config['marker_length'])  # markerLength
    square_size = float(board_config['square_size'])      # squareLength
    board_size = board_config['board_size']               # [rows, cols]

    squaresY = int(board_size[0])
    squaresX = int(board_size[1])

    aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
    board = cv2.aruco.CharucoBoard((squaresX, squaresY), square_size, marker_length, aruco_dict)
    return board, aruco_dict


def detect_aruco_board(image_folder: str, board_config: dict):
    """
    OpenCV 4.12：用 ArucoDetector 检测 marker，
    然后把“每个 marker 的4个角点”拼成 solvePnP 所需的 object/image points。
    返回：used_paths, object_points_list, image_points_list
    """
    dictionary_name = board_config['dictionary']
    marker_length = float(board_config['marker_length'])
    square_size = float(board_config['square_size'])
    board_size = board_config['board_size']
    squaresX, squaresY = get_squares_xy(board_size)

    aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)

    all_paths = glob.glob(os.path.join(image_folder, '*'))
    image_paths = sorted(all_paths, key=extract_index_from_path)

    used_paths = []
    object_points_list = []
    image_points_list = []

    margin = 0.5 * (square_size - marker_length)  # marker 在方块内的留白

    for p in image_paths:
        img = cv2.imread(p)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = detector.detectMarkers(gray)
        if ids is None or len(ids) == 0:
            print(f"Warning: No ArUco markers found in {p}")
            continue

        obj_pts = []
        img_pts = []

        for mk_corners, mk_id in zip(corners, ids.flatten()):
            pos = id_to_square_rc(int(mk_id), squaresX, squaresY)
            if pos is None:
                continue
            r, c = pos

            # marker 所在方块左上角（板坐标系：x右、y下、z=0）
            x0 = c * square_size + margin
            y0 = r * square_size + margin
            L = marker_length

            # 这个 marker 的 4 个角点 3D（与 detectMarkers 返回角点顺序一致：TL, TR, BR, BL）
            obj_marker = np.array([
                [x0,     y0,     0.0],
                [x0 + L, y0,     0.0],
                [x0 + L, y0 + L, 0.0],
                [x0,     y0 + L, 0.0],
            ], dtype=np.float64)

            img_marker = mk_corners.reshape(4, 2).astype(np.float64)

            obj_pts.append(obj_marker)
            img_pts.append(img_marker)

        if len(obj_pts) < 2:
            # 少于2个 marker（<8个点）很不稳，建议跳过
            print(f"Warning: Too few markers for PnP in {p} (markers={len(obj_pts)})")
            continue

        obj_pts = np.vstack(obj_pts)  # (N*4, 3)
        img_pts = np.vstack(img_pts)  # (N*4, 2)

        used_paths.append(p)
        object_points_list.append(obj_pts)
        image_points_list.append(img_pts)

    return used_paths, object_points_list, image_points_list


def calibrate_intrinsics(object_points: List[np.ndarray], image_points: List[np.ndarray], image_size: Tuple[int, int]):
    """
    使用 cv2.calibrateCamera 估计相机内参和畸变。如果已有内参可跳过。
    返回 (camera_matrix, dist_coeffs, rvecs, tvecs)
    """
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        object_points, image_points, image_size, None, None
    )
    if not ret:
        raise RuntimeError('calibrateCamera failed')
    return camera_matrix, dist_coeffs, rvecs, tvecs


def compute_extrinsics_per_image(
    object_points: List[np.ndarray],
    image_points: List[np.ndarray],
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray
):
    """
    对每张图像使用 solvePnP 计算 target->cam 的外参（R,t），返回列表的 (R_mat, tvec(3,1))
    注意：这里得到的是 ^C T_T （X_cam = R * X_target + t）
    """
    R_list = []
    t_list = []
    for objp, imgp in zip(object_points, image_points):
        objp2 = objp.reshape(-1, 3).astype(np.float64)
        imgp2 = imgp.reshape(-1, 2).astype(np.float64)

        success, rvec, tvec = cv2.solvePnP(objp2, imgp2, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
        if not success:
            raise RuntimeError('solvePnP failed for one image')

        R, _ = cv2.Rodrigues(rvec)
        R_list.append(R)
        t_list.append(tvec.reshape(3, 1))
    return R_list, t_list


def align_robot_poses_with_used_paths(robot_poses: List[np.ndarray], used_paths: List[str]) -> List[np.ndarray]:
    """
    用图片文件名里的数字索引去选取对应行的机器人位姿，避免丢帧导致错配。
    自动兼容 0-based / 1-based 索引。
    """
    if len(robot_poses) == 0 or len(used_paths) == 0:
        return []

    idxs = [extract_index_from_path(p) for p in used_paths]
    max_idx = max(idxs)
    min_idx = min(idxs)

    # 猜测索引从 0 还是 1 开始
    # 情况1：0-based：最大索引 <= len-1
    if max_idx <= len(robot_poses) - 1:
        base = 0
    # 情况2：1-based：最大索引 <= len 且最小索引 >= 1
    elif max_idx <= len(robot_poses) and min_idx >= 1:
        base = 1
    else:
        # 实在猜不出来就退化成顺序截断（不推荐，但比直接报错强）
        print("[Warning] Cannot infer image index base (0/1). Fallback to sequential pairing.")
        n = min(len(robot_poses), len(used_paths))
        return robot_poses[:n]

    selected = []
    for p, idx in zip(used_paths, idxs):
        j = idx - base
        if 0 <= j < len(robot_poses):
            selected.append(robot_poses[j])
        else:
            print(f"[Warning] Image {p} index {idx} out of range for robot poses (len={len(robot_poses)})")

    return selected


def hand_eye_calibration(robot_poses: List[np.ndarray], R_target2cam: List[np.ndarray], t_target2cam: List[np.ndarray]):
    """
    robot_poses：base->gripper
    R_target2cam/t_target2cam：PnP 得到的 ^C T_T（target->cam）
    返回：T_cam2gripper（camera->gripper）
    """
    n = min(len(robot_poses), len(R_target2cam))
    robot_poses = robot_poses[:n]
    R_target2cam = R_target2cam[:n]
    t_target2cam = t_target2cam[:n]

    # calibrateHandEye 需要：gripper->base，所以要把 base->gripper 取逆
    R_gripper2base = []
    t_gripper2base = []
    for T_BG in robot_poses:
        T_GB = inv_T(T_BG)
        R_gripper2base.append(T_GB[:3, :3])
        t_gripper2base.append(T_GB[:3, 3].reshape(3, 1))

    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        R_gripper2base, t_gripper2base,
        R_target2cam, t_target2cam,
        method=cv2.CALIB_HAND_EYE_TSAI
    )

    T_cam2gripper = np.eye(4)
    T_cam2gripper[:3, :3] = R_cam2gripper
    T_cam2gripper[:3, 3] = t_cam2gripper.reshape(3)
    return T_cam2gripper



def visualize_aruco_detection(image_folder: str, used_paths: List[str], board_config: dict,
                              output_folder: str, camera_matrix=None, dist_coeffs=None):
    """
    可视化：画出检测到的 marker；如果提供了内参，还画坐标轴（PnP 解出来的板姿态）
    """
    dictionary_name = board_config['dictionary']
    aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
    detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())

    os.makedirs(output_folder, exist_ok=True)
    print(f'[Info] 保存ArUco检测结果到 {output_folder}...')

    for p in used_paths:
        img = cv2.imread(p)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = detector.detectMarkers(gray)

        vis = img.copy()
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(vis, corners, ids)

        fname = os.path.basename(p)
        out_path = os.path.join(output_folder, f'detected_{fname}')
        cv2.imwrite(out_path, vis)

    print(f'[Info] 保存完成：{len(used_paths)} 张图片已标注并保存')

def save_transform_yaml(path: str, T: np.ndarray):
    """将 4x4 变换矩阵保存为 YAML 文件（平铺形式）"""
    data = {'transform': T.flatten().tolist()}
    with open(path, 'w') as f:
        yaml.safe_dump(data, f)


def main(config_path='config_lc.yaml', image_folder='lc_imgs', robot_poses_file='robot_pos.txt'):
    # 读取配置
    config = load_config(config_path)
    board_config = config['calibration_board']
    board_type = board_config.get('type', 'aruco')  # 兼容原配置

    # 你这块板是 ChArUco，这里不强制改 YAML，但内部用 CharucoBoard
    if board_type not in ('aruco', 'charuco'):
        raise ValueError(f"Unsupported board type: {board_type}. Use 'aruco' or 'charuco'.")

    print(f"Detecting ChArUco board points in {image_folder}...")
    used_paths, object_points_list, image_points_list = detect_aruco_board(image_folder, board_config)

    if len(used_paths) == 0:
        print('No valid ChArUco detections found. Abort')
        return

    # 读取/标定内参
    cam_cfg = config.get('camera', {})
    camera_matrix = None
    dist_coeffs = None

    have_intrinsics = False
    try:
        fx = cam_cfg['matrix']['fx']
        fy = cam_cfg['matrix']['fy']
        cx = cam_cfg['matrix']['cx']
        cy = cam_cfg['matrix']['cy']
        camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
        dist_coeffs = np.array(cam_cfg.get('distortion_coeffs', [0, 0, 0, 0, 0]), dtype=np.float64)
        R_target2cam_list, t_target2cam_list = compute_extrinsics_per_image(
            object_points_list, image_points_list, camera_matrix, dist_coeffs)
        have_intrinsics = True
        print('[Info] 使用配置文件中的相机内参')
    except Exception:
        have_intrinsics = False

    if not have_intrinsics:
        sample_img = cv2.imread(used_paths[0])
        image_size = (sample_img.shape[1], sample_img.shape[0])  # (w, h)
        print('[Info] 标定相机内参...')
        camera_matrix, dist_coeffs, _, _ = calibrate_intrinsics(object_points_list, image_points_list, image_size)
        print('Estimated camera matrix:\n', camera_matrix)
        print('Estimated dist coeffs:\n', dist_coeffs)

    # 可视化检测结果（此时已有内参，可画轴）
    visualize_aruco_detection(image_folder, used_paths, board_config, DEFAULT_OUTPUT_DIR, camera_matrix, dist_coeffs)

    # 用 solvePnP 计算每张图的 ^C T_T
    print('[Info] solvePnP 计算每张图的 target->cam 外参...')
    R_target2cam_list, t_target2cam_list = compute_extrinsics_per_image(
        object_points_list, image_points_list, camera_matrix, dist_coeffs
    )

    # 读取机器人位姿，并按图片索引对齐（避免丢帧错配）
    robot_poses_all = load_robot_poses(robot_poses_file)
    if len(robot_poses_all) == 0:
        print('No robot poses found. Abort')
        return

    robot_poses = align_robot_poses_with_used_paths(robot_poses_all, used_paths)
    n = min(len(robot_poses), len(R_target2cam_list))
    robot_poses = robot_poses[:n]
    R_target2cam_list = R_target2cam_list[:n]
    t_target2cam_list = t_target2cam_list[:n]

    if n < 10:
        print(f"[Warning] 有效配对样本太少（{n}），建议采集更多姿态（>=15-30）")

    # 手眼标定
    print('[Info] 执行手眼标定 (calibrateHandEye)')
    T_cam2gripper = hand_eye_calibration(robot_poses, R_target2cam_list, t_target2cam_list)

    print('\nResult: T_cam2gripper (camera -> gripper / TCP)')
    print(T_cam2gripper)

    # 保存结果
    out_dir = DEFAULT_OUTPUT_DIR
    os.makedirs(out_dir, exist_ok=True)
    save_transform_yaml(os.path.join(out_dir, 'T_cam2gripper.yaml'), T_cam2gripper)
    print(f'[Info] 结果已保存到 {out_dir}')
    print(f'[Info] 检测到 {len(used_paths)} 张有效的ChArUco图片，配对用于手眼的数量：{n}')


if __name__ == '__main__':
    main(config_path=DEFAULT_CONFIG_PATH, image_folder=DEFAULT_IMAGE_FOLDER, robot_poses_file=DEFAULT_ROBOT_POSES_FILE)
