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
DEFAULT_CONFIG_PATH = 'camera/config_lc.yaml'
DEFAULT_IMAGE_FOLDER = 'camera/rc_imgs/'
DEFAULT_OUTPUT_DIR = 'camera/calib_output/'
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
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        found = False
        corners = None
        # try the "SB" detector first if available
        try:
            res = cv2.findChessboardCornersSB(gray, chessboard_size)
            if isinstance(res, tuple):
                found = bool(res[0])
                corners = res[1] if found and len(res) > 1 else None
            else:
                found = bool(res)
        except Exception:
            found = False

        if not found:
            flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
            ret, corners = cv2.findChessboardCorners(gray, chessboard_size, flags)
            found = bool(ret)

        if found and corners is not None:
            term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), term)
            image_points.append(corners)
            used_paths.append(p)
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
    out = {
        'camera_matrix': camera_matrix.tolist(),
        'dist_coeffs': dist_coeffs.flatten().tolist(),
        'reprojection_error': float(mean_error),
        'num_views': int(len(object_points)),
    }
    out_path = os.path.join(output_dir, 'intrinsics.yaml')
    with open(out_path, 'w') as f:
        yaml.safe_dump(out, f)

    print('Calibration done')
    print('Camera matrix:\n', camera_matrix)
    print('Distortion coeffs:', dist_coeffs.flatten())
    print('Mean reprojection error:', mean_error)
    print('Saved intrinsics to', out_path)
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
