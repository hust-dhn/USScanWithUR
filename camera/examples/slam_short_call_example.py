from sys_path import *
from api.library_loader import *
from api.errors import ErrorCode, Error
from api.inu_sensor import InuSensor, ConnectionState, create_started_sensor
from api.slam import SlamStream, SlamFrame, SlamTransformationsParams, DebugTraceMode
from api.shared import StreamType
from api.base_stream import BaseStream
import time
if sys.platform == 'linux':
    import signal

# #########################################################  main #####################################################
if __name__ == "__main__":
    def _sensor_callback_func(self, sensor: InuSensor, connection_state: ConnectionState, error: Error) -> None:
        print(f'Sensor callback ConnectionState={connection_state} Error={error}')


    def _print_frame(frame: SlamFrame) -> None:
        if frame is None:
            print('Invalid frame ')
        else:
            print(f"\t---------- {frame} frame index {frame.frame_index} time {frame.time_stamp} ----------------")
            print(f"\t State {frame.state} InternalState {frame.internal_state}")
            print("\t Pose4x4BodyToWorld")
            for b2w in frame.pose4x4body2world:
                print("\t\t{}".format(b2w))
            print("\t Pose4x4WorldToBody")
            for w2b in frame.pose4x4world2body:
                print(f"\t\t{w2b}")
            print("\t------------------------------------------------------------------------------------------")


    def _slam_callback_func(stream: SlamStream, frame: SlamFrame, error: Error) -> None:
        if error is None:
            print(f'Undefined error in {type(stream)} ')
        elif error.code != ErrorCode.STATUS_OK:
            print(f'{type(stream)} stream callback Error = {error.code} {error.description}')
        else:
            _print_frame(frame)

    print(f"Start test application")
    sensor, channels_size, hw_information = create_started_sensor(_sensor_callback_func)
    print(f"{type(sensor)} initialization finished")
    exposeMap = True
    debugTrace = DebugTraceMode.ENABLE
    params = SlamTransformationsParams()
    stream = sensor.create_started_stream(StreamType.SLAM, BaseStream.DEFAULT_CHANNEL_ID,
                                          _slam_callback_func, params,
                                          exposeMap, debugTrace)
    sensor.record(r"C:\tmp\test", "TEST")
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
