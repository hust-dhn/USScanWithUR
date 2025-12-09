from sys_path import *
from api.library_loader import *
from api.errors import ErrorCode, Error
from api.inu_sensor import InuSensor, ConnectionState, DeviceParams, SensorResolution, ChannelDimensions, CropParams
from api.image_frame import ImageFrame
from api.depth_stream import DepthStream
from api.shared import StreamType, ImageFormat
from api.hw_info import ChannelControlParams
import time
if sys.platform == 'linux':
    import signal
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

    def show_frame(self, frame: ImageFrame)-> None:
        if frame is None or frame.width == 0 or frame.height == 0 or frame.bytes_per_pixel == 0:
            # raise Exception("Sorry, 'Empty Frame in show_frame")
            print("Sorry, 'Empty Frame in show_frame")
            return
        if self._camera is None:
            self._camera = cv2.VideoCapture(0)
            cv2.namedWindow(self._win_name, cv2.WINDOW_NORMAL)
        # right_frame.Save("C:\\tmp\\test.tiff")
        form = frame.format
        if form == ImageFormat.DEPTH or form == ImageFormat.DISPARITY:
            img = frame.buffer
            img = img * 255.0 / 8192.0  # scaling_factor
            img = np.uint8(img)
            img_right = cv2.applyColorMap(img, cv2.COLORMAP_HOT)
            cv2.namedWindow(self._win_name, cv2.WINDOW_NORMAL)
            print(f"resizeWindow( {self._win_name} , {frame.width}, {frame.height} )")
            cv2.resizeWindow(self._win_name, frame.width, frame.height)
            print(f"imshow")
            cv2.imshow(self._win_name, img)
            print(f"waitKey")
            cv2.waitKey(1)
            print(f"finish")
        else:
            # raise Exception(f"Sorry, 'Unsupported Image { form } format in show_frame")
            print(f"Sorry, 'Unsupported Image { form } format in show_frame")
            return

        if self._stop_show:
            self._camera.release()
            cv2.destroyWindow(self._win_name)
            # cv2.destroyAllWindows()
            self._camera = None


# #########################################################  main #####################################################
if __name__ == "__main__":
    def _sensor_callback_func(self, sensor: InuSensor, connection_state: ConnectionState, error: Error) -> None:
        print(f'Sensor callback ConnectionState={connection_state} Error={error}')


    def _print_frame(frame: ImageFrame) -> None:
        if frame is None:
            print('Invalid Depth frame')
        else:
            print(
                f"\t------------------------------------- Depth {frame.frame_index} ---------------------------------")
            print(f"{type(frame)} : Format {frame.format}, Height {frame.height}, Width{frame.width}, "
                  f"BytesPerPixel {frame.bytes_per_pixel}")
            print("\t------------------------------------------------------------------------------------------")


    def _depth_callback_func(stream: DepthStream, frame: ImageFrame, error: Error) -> None:
        if error is None:
            print(f'Undefined error in {type(stream)}')
        elif error.code != ErrorCode.STATUS_OK:
            print(f'{type(stream)} callback Error = {error.code} {error.description}')
        else:
            _print_frame(frame)
            viewer.show_frame(frame)

    print(f"Start test application")
    sensor = InuSensor()
    device_params = DeviceParams()
    device_params.resolution = SensorResolution.BINNING
    device_params.fps = 15
    hw_information, dpe = sensor.init(device_params)
    channelDepth = 3
    channelRGB = 4
    channelDimensions = ChannelDimensions()
    channelDimensions.width = 320
    channelDimensions.height = 480
    channelDimensions.buffer_width = channelDimensions.width
    channelDimensions.buffer_height = channelDimensions.height
    channelDimensions.start_x = 400
    channelDimensions.start_y = 300
    sensor.set_channel_dimensions(channelRGB, channelDimensions)
    # cropParams = CropParams()
    # cropParams.width = 640
    # cropParams.height = 480
    # cropParams.start_x = 400
    # cropParams.start_y = 300
    # sensor.set_channel_cropping(channelRGB, cropParams)
    channel_control_params_dict = {}
    for ch in hw_information.channels:
        if hw_information.channels[ch].ControlParams.SelectedImageFormat == ImageFormat.DEPTH:
            channel_control_params = ChannelControlParams(hw_information.channels[ch].ControlParams)
            channel_control_params.activate_registered_depth = True
            channel_control_params_dict[ch] = channel_control_params
    channels_size = sensor.start(channel_control_params_dict)
    sensor.register = _sensor_callback_func
    print(f"Sensor initialization finished")
    stream = sensor.create_stream(StreamType.DEPTH)
    stream.init()
    stream.start()
    # stream.register = _depth_callback_func
    print("Dept stream initialization finished")
    disparity = stream.disparity_params
    disparity.max_distance = 1000
    stream.disparity_params = disparity
    disparity_get = stream.disparity_params
    print(f"{type(stream)} set max_distanse to {disparity_get.max_distance}")
    viewer = ViewerCV2(f"{type(stream)}")
    print(f"{type(stream)} viewer created")
    for i in range(0, 5000):
        try:
            frame = stream.frame
            if frame is None :
                print(f"{type(stream)}.frame failed")
                continue
            _print_frame(frame)
            viewer.show_frame(frame)
            print(f"iteration {i}")
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
