import time

from sys_path import *
from api.library_loader import *
from api.errors import ErrorCode, Error
from api.inu_sensor import InuSensor, ConnectionState, SensorResolution
from api.hw_info import ChannelControlParams
from api.image_frame import ImageFrame
from api.image_stream import ImageStream
from api.shared import StreamType

if sys.platform == 'linux':
    import signal

interval = 1.0
test_time = 10
channel = 4  # 9, 4
mode = SensorResolution.BINNING.value
fps = 5

start_time = 0
frame_count = 0


# #########################################################  main #####################################################
if __name__ == "__main__":
    def _sensor_callback_func(self, sensor: InuSensor, connection_state: ConnectionState, error: Error) -> None:
        print(f'Sensor callback ConnectionState={connection_state} Error={error}')

    def _print_frame(stream: ImageStream, frame: ImageFrame) -> None:
        global start_time, frame_count, msg_queue

        if frame is None:
            print('Invalid Image frame')
        else:
            # print(
            #     f"\t------------------------------------- Image {frame.frame_index} ---------------------------------")
            # print(f"Frame Type: {type(frame)}, Format: {frame.format}, Width: {frame.width}, Height: {frame.height}, "
            #       f"BytesPerPixel {frame.bytes_per_pixel}")
            # print("\t------------------------------------------------------------------------------------------")

            frame_count += 1
            time_diff = time.perf_counter() - start_time
            if time_diff > interval:
                start_time = time.perf_counter()
                fps = frame_count / time_diff
                print(f"Frame Type: {type(frame)}, Channel: {stream.channel_id}, Format: {frame.format}, Width: {frame.width}, Height: {frame.height}, FPS: {fps}")
                frame_count = 0
                start_time = time.perf_counter()

    def _image_callback_func(stream: ImageStream, frame: ImageFrame, error: Error) -> None:
        if error is None:
            print(f'Undefined error in {type(stream)}')
        elif error.code != ErrorCode.STATUS_OK:
            print(f'{type(stream)} callback Error = {error.code} {error.description}')
        else:
            _print_frame(stream, frame)

    print(f"Start test application")
    sensor = InuSensor()

    # device_params = DeviceParams()
    # device_params.resolution = ESensorResolution.ESensorResolution(mode)
    # device_params.fps = fps
    # hw_information, dpe = sensor.init(device_params)
    hw_information, dpe = sensor.init()

    for ch in hw_information.channels:
        print(f'channel: {ch}, resolution: {hw_information.channels[ch].ControlParams.SensorRes}, fps: {hw_information.channels[ch].ControlParams.FPS}')

    channel_control_params = ChannelControlParams(hw_information.channels[channel].ControlParams)
    channel_control_params.fps = fps
    channel_control_params.sensor_res = mode
    channel_control_params_dict = {channel: channel_control_params}
    channels_size = sensor.start(channel_control_params_dict)
    sensor.register = _sensor_callback_func
    print(f"Sensor initialization finished")

    for ch in channel_control_params_dict:
        print(f'channel: {ch}, resolution: {channel_control_params_dict[ch].sensor_res}, '
              f'fps: {channel_control_params_dict[ch].fps}')

    start_time = time.perf_counter()

    stream = sensor.create_stream(StreamType.GENERAL_CAMERA, channel)
    stream.init()
    stream.start()
    stream.register = _image_callback_func
    print("Image stream initialization finished")
    print(f'Image stream channel: {stream.channel_id}')

    for i in range(test_time):
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            break
    print(f"Finish test application")

    stream.register = None
    stream.stop()
    stream.terminate()
    print(f"{type(stream)} Stopped")

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
