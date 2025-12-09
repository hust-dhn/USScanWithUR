from sys_path import *
from api.library_loader import *
from api.errors import ErrorCode, Error
from api.inu_sensor import InuSensor, ConnectionState, create_started_sensor, DeviceParams
from api.image_stream import ImageStream
from api.image_frame import ImageFrame
from api.depth_stream import DepthStream
from api.stereo import StereoFrame, StereoStream
from api.shared import StreamType, ImageFormat
from api.super_frame import SuperFrame, SuperFramesSyncStrategy, SuperFramesStrategy
import time
if sys.platform == 'linux':
    import signal
import cv2
import numpy as np
from typing import Union

_super_frame = None


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


    # streams
    _depth_stream = None
    _stereo_stream = None
    _general_camera_stream = None
    # viewers
    _depth_viewer = None;
    _stereo_viewer = None
    _general_camera_viewer = None


    def _print_stereo_frame(frame: StereoFrame, stream: StereoStream) -> None:
        if frame is None:
            print('Invalid Stereo frame')
        else:
            print(f"\t------- {type(frame)} from {type(stream)} index {frame.frame_index} -------")
            right = frame.right_frame
            if right is not None:
                print(f"Right Frame: Format {right.format}, Height {right.height},"
                  f" Width{right.width}, BytesPerPixel {right.bytes_per_pixel}")
            left = frame.left_frame
            if left is not None:
                print(f"Left Frame: Format {left.format}, Height {left.height},"
                  f" Width{left.width}, BytesPerPixel {left.bytes_per_pixel}")


    def _print_image_frame(frame: ImageFrame, stream: Union[DepthStream, ImageStream] ) -> None:
        if frame is None:
            print('Invalid Image frame')
        else:
            print(f"\t------- {type(frame)} from {type(stream)} index {frame.frame_index} -------")
            print(f"{type(frame)} : Format {frame.format}, Height {frame.height}, Width{frame.width}, "
                  f"BytesPerPixel {frame.bytes_per_pixel}")

    def perform_frame(frame: SuperFrame) -> None:
        print(f"\t------------------------ {type(frame)}  ------------------------")
        if _depth_stream is not None:
            depth_frame = frame.get_frame(_depth_stream)
            if depth_frame is not None:
                _print_image_frame(depth_frame, _depth_stream)
                if _depth_viewer is not None:
                    _depth_viewer.show_frame(depth_frame)
        if _stereo_stream is not None:
            stereo_frame = frame.get_frame(_stereo_stream)
            if stereo_frame is not None:
                _print_stereo_frame(stereo_frame, _stereo_stream)
                if _stereo_viewer is not None:
                    _stereo_viewer.show_frame(stereo_frame.right_frame, stereo_frame.left_frame)
        if _general_camera_stream is not None:
            general_camera_frame = frame.get_frame(_general_camera_stream)
            if general_camera_frame is not None:
                _print_image_frame(general_camera_frame, _general_camera_stream)
                if _general_camera_viewer is not None:
                    _general_camera_viewer.show_frame(general_camera_frame)
        print(f"\t---------------------------------------------------------------")


    def _superframe_callback_func(frame: SuperFrame, error: Error) -> None:
        global _super_frame
        if error is None:
            print(f'Undefined error.')
        elif error.code != ErrorCode.STATUS_OK:
            print(f'Callback Error = {error.code} {error.description}')
        else:
            if frame is None:
                print(f'_superframe_callback_func invalid frame')
            else:
                # _super_frame = frame
                perform_frame(frame)


    print(f"Start test application")
    deviceParams = DeviceParams()
    deviceParams.fps = 30
    sensor, channels_size, hw_information = create_started_sensor(_sensor_callback_func, deviceParams)
    print(f"Sensor initialization finished")
    _depth_stream = sensor.create_started_stream(StreamType.DEPTH)
    if _depth_stream is not None:
        _depth_viewer = ViewerCV2(f"{type(_depth_stream)}")
        print(f"{type(_depth_stream)} viewer created")
        print("Dept stream initialization finished")
    _stereo_stream = sensor.create_started_stream(StreamType.STEREO)
    if _stereo_stream is not None:
        _stereo_viewer = ViewerCV2(f"{type(_stereo_stream)}")
        print(f"{type(_stereo_stream)} viewer created")
        print("Stereo stream initialization finished")
    _general_camera_stream = sensor.create_started_stream(StreamType.GENERAL_CAMERA)
    if _general_camera_stream is not None:
        _general_camera_viewer = ViewerCV2(f"{type(_general_camera_stream)}")
        print(f"{type(_general_camera_stream)} viewer created")
        print("General camera stream initialization finished")
    streams = set()
    if _depth_stream is not None:
        streams.add(_depth_stream)
    if _stereo_stream is not None:
        streams.add(_stereo_stream)
    if _general_camera_stream is not None:
        streams.add(_general_camera_stream)
    strategy = SuperFramesStrategy()
    strategy.strategy_type = SuperFramesSyncStrategy.AUTO
    strategy.max_threshold = 50
    sensor.superframe_register(_superframe_callback_func, streams, strategy)
    print(f"superframe_register finish")
    for i in range(0, 5000):
        try:
            print(f"iteration {i}")
            time.sleep(1)
            # if _super_frame is not None:
            #     perform_frame(_super_frame)
            #     _super_frame = None
        except KeyboardInterrupt:
            break
    print(f"Finish test application")
    _depth_stream.stop()
    _depth_stream.terminate()
    print(f"{type(_depth_stream)} Stopped")
    _stereo_stream.stop()
    _stereo_stream.terminate()
    print(f"{type(_stereo_stream)} Stopped")
    _general_camera_stream.stop()
    _general_camera_stream.terminate()
    print(f"{type(_general_camera_stream)} Stopped")
    sensor.register = None
    sensor.stop()
    sensor.terminate()
    print(f'Sensor stopped')
    if sys.platform == 'win32':
        os._exit(0)
    elif sys.platform == 'linux':
        os.kill(os.getpid(), signal.SIGINT)
    else:
        sys.exit(0)
