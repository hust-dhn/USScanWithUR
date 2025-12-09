from sys_path import *
from api.library_loader import *
from api.errors import ErrorCode, Error
from api.inu_sensor import InuSensor, ConnectionState, CropParams, ChannelDimensions
from api.image_frame import ImageFrame
from api.image_stream import ImageStream, ImageOutputFormat, ImagePostProcessing
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
            print(f"{type(frame)} : Format {frame.format}, Height {frame.height}, Width {frame.width}, "
                  f"BytesPerPixel {frame.bytes_per_pixel}")
            print("\t------------------------------------------------------------------------------------------")


    def _gen_camera_callback_func(stream: ImageStream, frame: ImageFrame, error: Error) -> None:
        if error is None:
            print(f'Undefined error in {type(stream)} ')
        elif error.code != ErrorCode.STATUS_OK:
            print(f'{type(stream)} stream callback Error = {error.code} {error.description}')
        else:
            _print_frame(frame)

    print(f"Start test application")
    sensor = InuSensor()
    hw_information, dpe = sensor.init()
    channelRGB = 4
    # set channel dimensions case
    channelDimensions = ChannelDimensions()
    channelDimensions.width = 320
    channelDimensions.height = 480
    channelDimensions.buffer_width = channelDimensions.width
    channelDimensions.buffer_height = channelDimensions.height
    channelDimensions.start_x = 400
    channelDimensions.start_y = 300
    sensor.set_channel_dimensions(channelRGB, channelDimensions)
    # set channel cropping
    # cropParams = CropParams()
    # cropParams.width = 640
    # cropParams.height = 480
    # cropParams.start_x = 400
    # cropParams.start_y = 300
    # sensor.set_channel_cropping(channelRGB, cropParams)
    channels_size = sensor.start()
    sensor.register = _sensor_callback_func
    print(f"{type(sensor)} initialization finished")
    stream = sensor.create_started_stream(StreamType.GENERAL_CAMERA, channelRGB,
                                          _gen_camera_callback_func, ImageOutputFormat.RAW,
                                          ImagePostProcessing.NONE_PROCESSING)
    sensor.record(f"C:/tmp/test")
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
