#!/usr/bin/env python3
"""
Compute camera intrinsics from chessboard images.
Reads chessboard parameters from `camera/config_lc.yaml` by default, detects corners,
runs cv2.calibrateCamera and writes intrinsics to `camera/calib_output/intrinsics.yaml`.

Usage:
    python3 camera/calibration_intrinsics.py --images camera/lc_imgs --config camera/config_lc.yaml

"""

import os
import glob
import yaml
import argparse
import numpy as np
import cv2


# ===== User-editable defaults (change paths here) =====
BOOL_LC = False  # True for left camera, False for right camera

# LEFT
if BOOL_LC:
    DEFAULT_CONFIG_PATH = 'camera/config_lc.yaml'
    DEFAULT_IMAGE_FOLDER = 'camera/lc_imgs/'
    DEFAULT_OUTPUT_DIR = 'camera/calib_output/'
    DEFAULT_OUTPUT_INTRINSICS_FILE = 'intrinsics_lc.yaml'
# RIGHT
else:
    DEFAULT_CONFIG_PATH = 'camera/config_rc.yaml'
    DEFAULT_IMAGE_FOLDER = 'camera/rc_imgs/'
    DEFAULT_OUTPUT_DIR = 'camera/calib_output/'
    DEFAULT_OUTPUT_INTRINSICS_FILE = 'intrinsics_rc.yaml'

# ======================================================


def extract_index(p: str):
    name = os.path.basename(p)
    import re
    m = re.search(r"(\d+)", name)
    if m:
        return int(m.group(1))
    return 10**9


def load_config(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def generate_object_points(chessboard_size, square_size):
    objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
    objp *= square_size
    return objp


def detect_corners_on_images(image_folder, chessboard_size):
    all_paths = glob.glob(os.path.join(image_folder, '*'))
    paths = sorted(all_paths, key=extract_index)

    image_points = []
    object_points = []
    used_paths = []

    for p in paths:
        img = cv2.imread(p)
        if img is None:
            continue
        # preprocess gray image with CLAHE + blur to improve contrast
        gray_orig = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        def preprocess(g):
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            g2 = clahe.apply(g)
            g2 = cv2.GaussianBlur(g2, (5, 5), 0)
            return g2

        def try_detect(g):
            # try SB detector first
            try:
                res = cv2.findChessboardCornersSB(g, chessboard_size)
                if isinstance(res, tuple):
                    ok = bool(res[0])
                    corners_sb = res[1] if ok and len(res) > 1 else None
                    if ok and corners_sb is not None:
                        return True, corners_sb
                else:
                    if bool(res):
                        return True, res
            except Exception:
                pass

            # fallback to classic detector with flags
            flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
            try:
                ret, corners = cv2.findChessboardCorners(g, chessboard_size, flags)
                return bool(ret), corners
            except Exception:
                return False, None

        found = False
        corners = None

        # multi-scale search to handle small/large checkers and perspective
        scales = [1.0, 1.5, 0.8]
        for s in scales:
            if s == 1.0:
                g = preprocess(gray_orig)
            else:
                ws = int(gray_orig.shape[1] * s)
                hs = int(gray_orig.shape[0] * s)
                gscaled = cv2.resize(gray_orig, (ws, hs), interpolation=cv2.INTER_LINEAR)
                g = preprocess(gscaled)

            ok, cc = try_detect(g)
            if ok and cc is not None:
                # if scaled, convert corners back to original image coordinates
                if s != 1.0:
                    # cc may be Nx1x2
                    cc = cc.astype(np.float32) / float(s)
                # refine on original gray image
                term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                try:
                    cv2.cornerSubPix(gray_orig, cc, (11, 11), (-1, -1), term)
                except Exception:
                    pass
                corners = cc
                found = True
                break

        if found and corners is not None:
            image_points.append(corners)
            used_paths.append(p)
            # save debug visualization
            try:
                vis = img.copy()
                cv2.drawChessboardCorners(vis, chessboard_size, corners, True)
                dbg_dir = os.path.join(DEFAULT_OUTPUT_DIR, 'debug_images')
                os.makedirs(dbg_dir, exist_ok=True)
                fname = os.path.basename(p)
                outp = os.path.join(dbg_dir, f'detected_{fname}')
                cv2.imwrite(outp, vis)
            except Exception:
                pass
        else:
            print(f"Warning: chessboard not found in {p}")

    return used_paths, image_points


def calibrate_camera_from_chessboards(image_folder, chessboard_size, square_size, output_dir):
    os.makedirs(output_dir, exist_ok=True)
    objp = generate_object_points(chessboard_size, square_size)

    used_paths, image_points = detect_corners_on_images(image_folder, chessboard_size)
    if len(image_points) == 0:
        raise RuntimeError('No chessboard corners detected')

    object_points = [objp for _ in image_points]

    sample = cv2.imread(used_paths[0])
    image_size = (sample.shape[1], sample.shape[0])

    print(f"Calibrating with {len(image_points)} views, image size={image_size}...")
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(object_points, image_points, image_size, None, None)
    if not ret:
        raise RuntimeError('calibrateCamera failed')

    # compute reprojection error
    tot_error = 0
    for i in range(len(object_points)):
        imgpoints2, _ = cv2.projectPoints(object_points[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
        error = cv2.norm(image_points[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        tot_error += error
    mean_error = tot_error / len(object_points)

    # Save intrinsics
    # 保存为与 config_lc.yaml 类似的结构，便于后续读取
    fx = float(camera_matrix[0, 0])
    fy = float(camera_matrix[1, 1])
    cx = float(camera_matrix[0, 2])
    cy = float(camera_matrix[1, 2])

    out = {
        'camera': {
            'matrix': {
                'fx': fx,
                'fy': fy,
                'cx': cx,
                'cy': cy,
            },
            'distortion_coeffs': dist_coeffs.flatten().tolist()
        },
        'reprojection_error': float(mean_error),
        'num_views': int(len(object_points)),
    }

    # 也将棋盘信息写入输出（便于作为 config 使用）
    try:
        out['chessboard'] = {
            'size': list(chessboard_size),
            'square_size': float(square_size)
        }
    except Exception:
        pass

    out_path = os.path.join(output_dir, DEFAULT_OUTPUT_INTRINSICS_FILE)

    # 手动格式化输出，确保 distortion_coeffs 为内联数组样式，与 config_lc.yaml 保持一致
    dist_list = dist_coeffs.flatten().tolist()
    dist_str = ', '.join([f'{float(x):.16g}' for x in dist_list])

    lines = []
    lines.append('camera:')
    lines.append('  matrix:')
    lines.append(f'    fx: {fx}')
    lines.append(f'    fy: {fy}')
    lines.append(f'    cx: {cx}')
    lines.append(f'    cy: {cy}')
    lines.append(f'  distortion_coeffs: [{dist_str}]')

    # optional chessboard
    try:
        lines.append('chessboard:')
        lines.append(f'  size: [{chessboard_size[0]}, {chessboard_size[1]}]')
        lines.append(f'  square_size: {float(square_size)}')
    except Exception:
        pass

    lines.append(f'num_views: {int(len(object_points))}')
    lines.append(f'reprojection_error: {float(mean_error)}')

    with open(out_path, 'w') as f:
        f.write('\n'.join(lines) + '\n')

    print('Calibration done')
    print('Camera matrix:\n', camera_matrix)
    print('Distortion coeffs:', dist_coeffs.flatten())
    print('Mean reprojection error:', mean_error)
    print('Saved intrinsics (config style) to', out_path)
    return camera_matrix, dist_coeffs


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--images', '-i', default=DEFAULT_IMAGE_FOLDER, help='Image folder')
    parser.add_argument('--config', '-c', default=DEFAULT_CONFIG_PATH, help='Config YAML with chessboard params')
    parser.add_argument('--out', '-o', default=DEFAULT_OUTPUT_DIR, help='Output folder')
    args = parser.parse_args()

    cfg = load_config(args.config)
    cb = tuple(cfg['chessboard']['size'])
    sq = cfg['chessboard']['square_size']

    calibrate_camera_from_chessboards(args.images, cb, sq, args.out)
