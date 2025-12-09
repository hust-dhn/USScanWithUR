from sys_path import *
from api.library_loader import *
from api.errors import ErrorCode, Error
from api.inu_sensor import InuSensor, ConnectionState, DeviceParams, create_started_sensor
from api.image_frame import ImageFrame
from api.image_stream import ImageStream
from api.depth_stream import DepthStream
from api.stereo import StereoStream, StereoFrame
from api.shared import StreamType, ImageFormat
from typing import Union

import time
import threading
import cv2
import numpy as np

if sys.platform == 'linux':
    import signal

"""!
    This example demonstrates how you can work with two treads to acquire frames from depth and image streams.
    The “_stream_callback_func“ receives depth and image frames.
    Based on the stream type the frames are kept either on the global members “depth_frame“ or “rgb_frame“.
    Two threads simultaneously reads and displays the global frames.
"""


class ViewerCV2:
    _camera = None
    _win_name: str = None
    _stop_show: bool = False

    def __init__(self, win_name: str):
        self._win_name = win_name
        # self._camera = cv2.VideoCapture(0)
        # cv2.namedWindow(self._win_name, cv2.WINDOW_NORMAL)

    def show_frame(self, right_frame: ImageFrame, left_frame: ImageFrame = None) -> None:
        if right_frame is None or right_frame.width == 0 or right_frame.height == 0 or right_frame.bytes_per_pixel == 0:
            # raise Exception("Sorry, 'Empty Frame in show_frame")
            print("Sorry, 'Empty Frame in show_frame")
            return
        if self._camera is None:
            self._camera = cv2.VideoCapture(0)
            cv2.namedWindow(self._win_name, cv2.WINDOW_NORMAL)
        # right_frame.Save("C:\\tmp\\test.tiff")
        with_left = left_frame is not None and left_frame.height and left_frame.width and left_frame.bytes_per_pixel
        img_right = None
        img_left = None
        form = right_frame.format
        if form == ImageFormat.BGRA or form == ImageFormat.RGBA:
            img_right = right_frame.buffer
            if with_left:
                img_left = left_frame.buffer
        elif form == ImageFormat.DEPTH or form == ImageFormat.DISPARITY:
            img_right = right_frame.buffer
            img_right = img_right * 255.0 / 8192.0  # scaling_factor
            img_right = np.uint8(img_right)
            img_right = cv2.applyColorMap(img_right, cv2.COLORMAP_HOT)
        else:
            # raise Exception(f"Sorry, 'Unsupported Image { form } format in show_frame")
            print(f"Sorry, 'Unsupported Image {form} format in show_frame")
            return
        img = None
        if with_left:
            img = cv2.hconcat([img_left, img_right])
        else:
            img = img_right
        cv2.namedWindow(self._win_name, cv2.WINDOW_NORMAL)
        print(f"resizeWindow( {self._win_name} , {right_frame.width}, {right_frame.height} )")
        cv2.resizeWindow(self._win_name, right_frame.width, right_frame.height)
        cv2.imshow(self._win_name, img)
        cv2.waitKey(1)
        if self._stop_show:
            self._camera.release()
            cv2.destroyWindow(self._win_name)
            # cv2.destroyAllWindows()
            self._camera = None


run_threads = True

rgb_channel = 9
depth_channel = 5  # 5, 3

rgb_stream = None
depth_stream = None

rgb_idx = 0
depth_idx = 0

rgb_frame = None
depth_frame = None

# Condition object for synchronization (similar to std::condition_variable)
rgb_condition = threading.Condition()
depth_condition = threading.Condition()

# #########################################################  main #####################################################
if __name__ == "__main__":
    def _sensor_callback_func(self, sensor: InuSensor, connection_state: ConnectionState, error: Error) -> None:
        print(f'Sensor callback ConnectionState={connection_state} Error={error}')

    def _print_frame(st: Union[DepthStream, ImageStream, StereoStream], fr: Union[ImageFrame, StereoFrame]) -> None:
        if fr is None:
            print(f'Invalid Frame')
        elif st is None:
            print(f'Invalid Stream')
        else:
            print(f'Print: stream: {type(st)} frame: {type(fr)} Index: {fr.frame_index} Time: {fr.time_stamp}')

    def _show_depth():
        global depth_frame
        while run_threads:
            with depth_condition:  # Acquire the lock
                while depth_frame:  # Check if there's data to consume
                    print("Consumer: Waiting for depth_frame...")
                    _print_frame(depth_stream, depth_frame)
                    depth_viewer.show_frame(depth_frame)
                    depth_frame = None

    def _show_rgb():
        global rgb_frame
        while run_threads:
            with rgb_condition:  # Acquire the lock
                while rgb_frame:  # Check if there's data to consume
                    print("Consumer: Waiting for rgb_frame...")
                    _print_frame(rgb_stream, rgb_frame)
                    rgb_viewer.show_frame(rgb_frame)
                    rgb_frame = None

    def _stream_callback_func(stream: Union[ImageStream, DepthStream], frame: ImageFrame, error: Error) -> None:
        global rgb_frame
        global depth_frame
        if error is None:
            print(f'Undefined error in {type(stream)}')
        elif error.code != ErrorCode.STATUS_OK:
            print(f'{type(stream)} callback Error = {error.code} {error.description}')
        else:
            if type(stream) is DepthStream:
                with depth_condition:  # Acquire the lock
                    depth_frame = frame
                    print(f"Producer: Notifying depth_condition")
                    depth_condition.notify()  # Signal consumer (like std::condition_variable::notify_one)
            elif type(stream) is ImageStream:
                with rgb_condition:  # Acquire the lock
                    rgb_frame = frame
                    print(f"Producer: Notifying rgb_condition")
                    rgb_condition.notify()  # Signal consumer (like std::condition_variable::notify_one)
            else:
                print(f'Invalid streamer type {type(stream)}')

    print("Start test application")
    depth_viewer = ViewerCV2(f"DEPTH {depth_channel}")
    print(f"DEPTH viewer created")
    rgb_viewer = ViewerCV2(f"RGB {rgb_channel}")
    print("RGB viewer created")
    show_depth = threading.Thread(target=_show_depth)
    show_depth.start()
    show_rgb = threading.Thread(target=_show_rgb)
    show_rgb.start()
    print("Start show thread")
    device_params = DeviceParams()
    device_params.fps = 10
    sensor, channels_size, hw_information = create_started_sensor(_sensor_callback_func)
    print(f"Sensor initialization finished")
    depth_stream = sensor.create_started_stream(StreamType.DEPTH, depth_channel, _stream_callback_func)
    print(f"Depth stream initialization finished. Channel: {depth_stream.channel_id}")
    rgb_stream = sensor.create_started_stream(StreamType.GENERAL_CAMERA, rgb_channel, _stream_callback_func)
    print(f"Image stream initialization finished. Channel: {rgb_stream.channel_id}")

    for idx in range(0, 5000):
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            break

    run_threads = False
    with rgb_condition:  # Acquire the lock
        rgb_condition.notify()  # Signal consumer (like std::condition_variable::notify_one)
    with depth_condition:  # Acquire the lock
        depth_condition.notify()  # Signal consumer (like std::condition_variable::notify_one)
    rgb_stream.terminate()
    print(f"{type(rgb_stream)} Stopped")
    depth_stream.terminate()
    print(f"{type(depth_stream)} Stopped")
    print(f"Finish test application")
    sensor.terminate()
    print(f'Sensor stopped')
    if sys.platform == 'win32':
        os._exit(0)
    elif sys.platform == 'linux':
        os.kill(os.getpid(), signal.SIGINT)
    else:
        sys.exit(0)
