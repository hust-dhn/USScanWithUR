from sys_path import *
from api.library_loader import *
from api.errors import ErrorCode, Error
from api.inu_sensor import InuSensor, ConnectionState, DeviceParams, SensorResolution
from api.image_frame import ImageFrame
from api.depth_stream import DepthStream
from api.image_stream import ImageStream
from api.shared import StreamType, ImageFormat
import time
if sys.platform == 'linux':
    import signal

from typing import Union
import cv2
import numpy as np


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
    depthViewer = ViewerCV2(f"DEPTH")
    print(f"DEPTH viewer created")
    rgbViewer = ViewerCV2(f"RGB")
    print("RGB viewer created")

    def _sensor_callback_func(self, sensor: InuSensor, connection_state: ConnectionState, error: Error) -> None:
        print(f'Sensor callback ConnectionState={connection_state} Error={error}')

    def _print_frame(stream_name: str, frame: ImageFrame) -> None:
        if frame is None:
            print('Invalid {type(frame)} frame')
        else:
            print(
                f"\t------------------------------------- {stream_name} {frame.frame_index} -------------------------")
            print(f"{stream_name} : Format {frame.format}, Height {frame.height}, Width{frame.width}, "
                  f"BytesPerPixel {frame.bytes_per_pixel}")
            print("\t------------------------------------------------------------------------------------------")

    # def _depth_callback_func(stream: DepthStream, frame: ImageFrame, error: Error) -> None:
    #     if error is None:
    #         print(f'Undefined error in {type(stream)}')
    #     elif error.code != ErrorCode.STATUS_OK:
    #         print(f'{type(stream)} callback Error = {error.code} {error.description}')
    #     else:
    #         _print_frame("DEPTH", frame)
    #         depthViewer.show_frame(frame)

    # def _image_callback_func(stream: ImageStream, frame: ImageFrame, error: Error) -> None:
    #     if error is None:
    #         print(f'Undefined error in {type(stream)}')
    #     elif error.code != ErrorCode.STATUS_OK:
    #         print(f'{type(stream)} callback Error = {error.code} {error.description}')
    #     else:
    #         _print_frame("RGB", frame)
    #         rgbViewer.show_frame(frame)

    print(f"Start test application")
    sensor = InuSensor()
    device_params = DeviceParams()
    device_params.resolution = SensorResolution.BINNING
    device_params.fps = 15
    hw_information, dpe = sensor.init(device_params)
    channels_size = sensor.start()
    sensor.register = _sensor_callback_func
    print(f"Sensor initialization finished")
    channelRGB = 4
    rgbStream = sensor.create_stream(StreamType.GENERAL_CAMERA, channelRGB)
    rgbStream.init()
    rgbStream.start()
    # rgbStream.register = _image_callback_func
    channelDepth = 3
    depthStream = sensor.create_stream(StreamType.DEPTH, channelDepth)
    depthStream.init()
    depthStream.start()
    # depthStream.register = _depth_callback_func
    print("{type(depthStream)} initialization finished")
    disparity = depthStream.disparity_params
    disparity.max_distance = 1000
    depthStream.disparity_params = disparity
    disparity_get = depthStream.disparity_params
    print(f"{type(depthStream)} set max distance to {disparity_get.max_distance}")
    if sys.platform == 'linux':
        sensor.record("/tmp")
    elif sys.platform == 'win32':
        sensor.record("c:/tmp")
    # print("Recording ON")
    for i in range(0, 5000):
        try:
            depthFrame = depthStream.frame
            _print_frame("DEPTH", depthFrame)
            depthViewer.show_frame(depthFrame)
            rgbFrame = rgbStream.frame
            _print_frame("RGB", rgbFrame)
            rgbViewer.show_frame(rgbFrame)
            print(f"iteration {i}")
            time.sleep(1)
        except KeyboardInterrupt:
            break
    print(f"Finish test application")
    rgbStream.terminate()
    print(f"{type(rgbStream)} Stopped")
    depthStream.terminate()
    print(f"{type(depthStream)} Stopped")
    sensor.terminate()
    print(f'Sensor stopped')
    if sys.platform == 'win32':
        os._exit(0)
    elif sys.platform == 'linux':
        os.kill(os.getpid(), signal.SIGINT)
    else:
        sys.exit(0)

