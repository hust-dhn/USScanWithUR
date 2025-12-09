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

    rgb_channel_id = 4
    rgb_sensor_id = 2

    print(f"Start test application")
    sensor = InuSensor()
    hw_information, dpe = sensor.init()
    channels_size = sensor.start()
    sensor.register = _sensor_callback_func
    print(f"Sensor initialization finished")
    stream = sensor.create_stream(StreamType.GENERAL_CAMERA,rgb_channel_id)
    stream.init()
    stream.start()
    print("Image stream initialization finished")
    stereo_viewer = ViewerCV2(f"RGB {rgb_channel_id}")
    for i in range(0, 5000):
        try:
            #_print_frame(stream.frame)
            stereo_viewer.show_frame(stream.frame)
            print(f"iteration {i}")
            #_print_frame(stream.frame)
            print("Exposure params: ")
            print(sensor.control_params[rgb_sensor_id].ExposureTime)
            control_params = sensor.control_params[rgb_sensor_id]
            control_params.AutoControl = False
            control_params.ExposureTime = (i*2500)
            sensor.set_control_params(rgb_sensor_id, control_params)
            time.sleep(1)
        except KeyboardInterrupt:
            break
    print(f"Finish test application")
    stream.terminate()
    print(f"{type(stream)} Stopped")
    sensor.terminate()
    print(f'Sensor stopped')
    if sys.platform == 'win32':
        os._exit(0)
    elif sys.platform == 'linux':
        os.kill(os.getpid(), signal.SIGINT)
    else:
        sys.exit(0)

