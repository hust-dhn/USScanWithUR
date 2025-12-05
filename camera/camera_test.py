import threading
import time
import rtde_receive
import rtde_control
import numpy as np
import cv2
import yaml
from pathlib import Path
from datetime import datetime

# 相机参数
CAMERA_LC_INDEX = 0  # 左相机编号
CAMERA_RC_INDEX = 1  # 右相机编号
IMG_WIDTH = 640
IMG_HEIGHT = 640

left_camera = None
right_camera = None

def initialize_cameras():
    """初始化左右相机"""
    global left_camera, right_camera
    try:
        left_camera = cv2.VideoCapture(CAMERA_LC_INDEX)
        # right_camera = cv2.VideoCapture(CAMERA_RC_INDEX)
        
        print("==========1==========")
        # 设置相机分辨率
        left_camera.set(cv2.CAP_PROP_FRAME_WIDTH, IMG_WIDTH)
        left_camera.set(cv2.CAP_PROP_FRAME_HEIGHT, IMG_HEIGHT)
        # right_camera.set(cv2.CAP_PROP_FRAME_WIDTH, IMG_WIDTH)
        # right_camera.set(cv2.CAP_PROP_FRAME_HEIGHT, IMG_HEIGHT)
        
        # 检查相机是否成功打开
        # if not left_camera.isOpened() or not right_camera.isOpened():
        #     print("[Error] 无法打开相机")
        #     return False

        if not left_camera.isOpened():
            print("[Error] 无法打开相机")
            return False

        print(f"[Camera] 相机已初始化")
        print(f"[Camera] 分辨率: {IMG_WIDTH}x{IMG_HEIGHT}")
        return True
    except Exception as e:
        print(f"[Error] 相机初始化失败: {e}")
        return False


if __name__ == "__main__":
    initialize_cameras()