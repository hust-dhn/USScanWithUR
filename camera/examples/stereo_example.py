from sys_path import *
from api.library_loader import *
from api.errors import ErrorCode, Error
from api.inu_sensor import InuSensor, ConnectionState
from api.image_frame import ImageFrame
from api.stereo import StereoStream
from api.shared import StreamType
import time
from api.hw_info import InterleaveMode, ChannelControlParams
if sys.platform == 'linux':
    import signal

# #########################################################  main #####################################################

with_interlive = True

if __name__ == "__main__":
    def _sensor_callback_func(self, sensor: InuSensor, connection_state: ConnectionState, error: Error) -> None:
        print(f'Sensor callback ConnectionState={connection_state} Error={error}')


    def _print_frame(frame: ImageFrame) -> None:
        if frame is None:
            print('Invalid Stereo frame')
        else:
            print(
                f"\t------------------------------------- Stereo {frame.frame_index} ---------------------------------")
            print(f"{type(frame)}")
            print(f"Right Frame: Format {frame.right_frame.format}, Height {frame.right_frame.height},"
                  f" Width{frame.right_frame.width}, BytesPerPixel {frame.right_frame.bytes_per_pixel}")
            if with_interlive and frame.left_frame is not None:
                print(f"Left Frame: Format {frame.left_frame.format}, Height {frame.left_frame.height},"
                      f" Width{frame.left_frame.width}, BytesPerPixel {frame.left_frame.bytes_per_pixel}")
            print("\t------------------------------------------------------------------------------------------")


    def _stereo_callback_func(stream: StereoStream, frame: ImageFrame, error: Error) -> None:
        if error is None:
            print(f'Undefined error in {type(stream)}')
        elif error.code != ErrorCode.STATUS_OK:
            print(f'{type(stream)} callback Error = {error.code} {error.description}')
        else:
            _print_frame(frame)


    print(f"Start test application")
    sensor = InuSensor()
    hw_information, dpe = sensor.init()
    stereo_channel = 0
    channel_control_params = ChannelControlParams(hw_information.channels[stereo_channel].ControlParams)
    channel_control_params.interleave_mode = InterleaveMode.INTERLEAVE
    channel_control_params_dict = {stereo_channel: channel_control_params}
    channels_size = sensor.start(channel_control_params_dict)
    sensor.register = _sensor_callback_func
    print(f"Sensor initialization finished")
    stream = sensor.create_stream(StreamType.STEREO)
    stream.init()
    stream.start()
    stream.register = _stereo_callback_func
    print("Stereo stream initialization finished")
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

