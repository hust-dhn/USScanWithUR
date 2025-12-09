from sys_path import *
from api.library_loader import *
from api.errors import ErrorCode, Error
from api.inu_sensor import InuSensor, ConnectionState, create_started_sensor
from api.image_frame import ImageFrame
from api.image_stream import ImageStream
from api.depth_stream import DepthStream
from api.shared import StreamType, ImageFormat

import time
import threading
import cv2
import numpy as np
if sys.platform == 'linux':
    import signal

"""! The Cnn stream class initializer.
    This example demonstrates how to read frames from two different streams simultaneously using the frame property.
    
    Two threads: 
    1. run_image_stream: retrieves image frames using the frame property
    2. run_depth_stream: retrieves depth frames using the frame property
"""


rgb_channel = 9
rgb_stream = None
depth_channel = 5  # 5, 3
depth_stream = None
run_threads = True


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


# #########################################################  main #####################################################
if __name__ == "__main__":
    def _sensor_callback_func(self, sensor: InuSensor, connection_state: ConnectionState, error: Error) -> None:
        print(f'Sensor callback ConnectionState={connection_state} Error={error}')

    def run_image_stream() -> None:
        def _image_print_frame(stream: ImageStream, frame: ImageFrame) -> None:
            if frame is None:
                print('Invalid Image frame')
            else:
                print(f"Func: _image_print_frame, Frame Type: {type(frame)}, Channel: {stream.channel_id}, "
                      f"Index: {frame.frame_index} Time: {frame.time_stamp} Format: {frame.format}, Width: {frame.width},"
                      f" Height: {frame.height}")

        rgb_viewer = ViewerCV2(f"RGB {rgb_channel}")
        print("RGB viewer created")
        while run_threads:
            rgb_frame = rgb_stream.frame
            _image_print_frame(rgb_stream, rgb_frame)
            rgb_viewer.show_frame(rgb_frame)

    def run_depth_stream():
        def _depth_print_frame(stream: DepthStream, frame: ImageFrame) -> None:
            if frame is None:
                print('Invalid Image frame')
            else:
                print(f"Func: _depth_print_frame, Frame Type: {type(frame)}, Channel: {stream.channel_id}, "
                      f"Index: {frame.frame_index} Time: {frame.time_stamp} Format: {frame.format}, Width: {frame.width},"
                      f" Height: {frame.height}")

        depth_viewer = ViewerCV2(f"DEPTH {depth_channel}")
        print(f"DEPTH viewer created")
        while run_threads:
            depth_frame = depth_stream.frame
            _depth_print_frame(depth_stream, depth_frame)
            depth_viewer.show_frame(depth_frame)

    print(f"Start test application")
    sensor, channels_size, hw_information = create_started_sensor(_sensor_callback_func)
    print(f"Sensor initialization finished")
    # stereo_stream = sensor.create_started_stream(StreamType.STEREO, stereo_channel)
    # print("Stereo stream initialization finished. Channel: {stereo_stream.channel_id}")
    depth_stream = sensor.create_started_stream(StreamType.DEPTH, depth_channel)
    print("Depth stream initialization finished. Channel: {depth_stream.channel_id}")
    rgb_stream = sensor.create_started_stream(StreamType.GENERAL_CAMERA, rgb_channel)
    print("Image stream initialization finished. Channel: {rgb_stream.channel_id}")
    image_thread = threading.Thread(target=run_image_stream)
    depth_thread = threading.Thread(target=run_depth_stream)
    image_thread.start()
    depth_thread.start()
    image_thread.join()
    depth_thread.join()

    for idx in range(0, 5000):
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            break

    run_threads = False
    rgb_stream.terminate()
    print(f"{type(rgb_stream)} Stopped")
    depth_stream.terminate()
    print(f"{type(depth_stream)} Stopped")
    # stereo_stream.terminate()
    # print(f"{type(stereo_stream)} Stopped")
    print(f"Finish test application")
    sensor.terminate()
    print(f'Sensor stopped')
    if sys.platform == 'win32':
        os._exit(0)
    elif sys.platform == 'linux':
        os.kill(os.getpid(), signal.SIGINT)
    else:
        sys.exit(0)
