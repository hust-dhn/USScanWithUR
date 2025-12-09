from sys_path import *
from api.library_loader import *
from api.errors import ErrorCode, Error
from api.inu_sensor import InuSensor, ConnectionState
from api.image_frame import ImageFrame
from api.depth_stream import DepthStream
from api.shared import StreamType, ImageFormat
from api.hw_info import ChannelControlParams
import time
if sys.platform == 'linux':
    import signal

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

    print(f"Start test application")
    sensor = InuSensor()
    hw_information, dpe = sensor.init()
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
    stream.register = _depth_callback_func
    print("Dept stream initialization finished")
    for i in range(0, 5000):
        try:
            _print_frame(stream.frame)
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
