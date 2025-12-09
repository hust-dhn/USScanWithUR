from sys_path import *
from api.library_loader import *
from api.errors import ErrorCode, Error
from api.inu_sensor import InuSensor, ConnectionState, create_started_sensor
from api.image_frame import ImageFrame
from api.image_stream import ImageStream, ImageOutputFormat, ImagePostProcessing
from api.depth_stream import DepthPostProcessing
from api.shared import StreamType
from api.base_stream import BaseStream
import time

if sys.platform == 'linux':
    import signal

# #########################################################  main #####################################################
if __name__ == "__main__":
    def _sensor_callback_func(self, sensor: InuSensor, connection_state: ConnectionState, error: Error) -> None:
        print(f'Sensor callback ConnectionState={connection_state} Error={error}')


    def _print_frame(frame: ImageFrame) -> None:
        if frame is None:
            print('Invalid frame ')
        else:
            print(
                f"\t---------------------------- {type(frame)}  {frame.frame_index} ---------------------------")
            print(f"{type(frame)} : Format {frame.format}, Height {frame.height}, Width{frame.width}, BytesPerPixel"
                  f"{frame.bytes_per_pixel}")
            print("\t------------------------------------------------------------------------------------------")


    def _gen_camera_callback_func(stream: ImageStream, frame: ImageFrame, error: Error) -> None:
        if error is None:
            print(f'Undefined error in {type(stream)} ')
        elif error.code != ErrorCode.STATUS_OK:
            print(f'{type(stream)} stream callback Error = {error.code} {error.description}')
        else:
            _print_frame(frame)


    print(f"Start test application")
    sensor, channels_size, hw_information = create_started_sensor(_sensor_callback_func)
    print(f"{type(sensor)} initialization finished")
    depthRegChannel = 3  # BaseStream.DEFAULT_CHANNEL_ID
    stream = sensor.create_started_stream(StreamType.GENERAL_CAMERA_REGISTERED, BaseStream.DEFAULT_CHANNEL_ID,
                                          _gen_camera_callback_func, ImageOutputFormat.DEFAULT,
                                          ImagePostProcessing.NONE_PROCESSING, DepthPostProcessing.DEFAULT)
    disparity = stream.disparity_params
    print(f"{type(stream)} initialization finished")
    for i in range(0, 5000):
        try:
            # _print_frame(stream.frame)
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
