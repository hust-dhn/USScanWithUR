from sys_path import *
from api.library_loader import *
from api.errors import ErrorCode, Error
from api.inu_sensor import InuSensor, ConnectionState, create_started_sensor
from api.point_cloud import PointCloudStream, PointCloudFrame, PointCloudOutputFormat, VoxelFilterParams,\
    SegmentationParams
from api.shared import StreamType
from api.base_stream import BaseStream
import time
if sys.platform == 'linux':
    import signal

# #########################################################  main #####################################################
if __name__ == "__main__":
    def _sensor_callback_func(self, sensor: InuSensor, connection_state: ConnectionState, error: Error) -> None:
        print(f'Sensor callback ConnectionState={connection_state} Error={error}')

    def _print_frame(frame: PointCloudFrame) -> None:
        if frame is None:
            print('Invalid frame ')
        else:
            print(
                f'PointCloud :Format {frame.format}, NumOfPoints {frame.num_of_points} BytesPerPoint {frame.bytes_per_point}')
            points = frame.data
            if points.size == 0:
                print("\t\tEMPTY Points")
            if frame.format == PointCloudOutputFormat.FLOAT_POINTS:
                print("\t\tFloatPoints : ")
                i = 0
                while i < frame.num_of_points:
                    print(f"\t\t # {i} X {points[i * 3 + 0]} Y {points[i * 3 + 1]} Z {points[i * 3 + 2]}")
                    i = i + 1
            elif frame.format == PointCloudOutputFormat.SHORT_POINTS:
                print("\t\tShortPoints : ")
                i = 0
                while i < frame.num_of_points:
                    print(f"\t\t # {i} X {points[i * 3 + 0]} Y {points[i * 3 + 1]} Z {points[i * 3 + 2]}")
                    i = i + 1
            elif frame.format == PointCloudOutputFormat.FLOAT_POINTS_RGB:
                print("\t\tFloatPointsRGB : ")
                i = 0
                while i < frame.num_of_points:
                    rgba = convert(int(points[i * 3 + 3]))
                    print(f"\t\t # {i} X {points[i * 3 + 0]} Y {points[i * 3 + 1]} Z {points[i * 3 + 2]}"
                          f" R {rgba[0]} G {rgba[1]} B {rgba[2]} A {rgba[3]} ")
                    i = i + 1
            else:
                raise Exception("Sorry, Unsupported PointCloudFormat")


    def _point_cloud_callback_func(stream: PointCloudStream, frame: PointCloudFrame, error: Error) -> None:
        if error is None:
            print(f'Undefined error in {type(stream)} ')
        elif error.code != ErrorCode.STATUS_OK:
            print(f'{type(stream)} stream callback Error = {error.code} {error.description}')
        else:
            print(f'Frame printing')
            _print_frame(frame)

    print(f"Start test application")
    sensor, channels_size, hw_information = create_started_sensor(_sensor_callback_func)
    print(f"{type(sensor)} initialization finished")

#   params = VoxelFilterParams()
    stream = sensor.create_started_stream(StreamType.POINT_CLOUD, None, _point_cloud_callback_func)
#   stream = sensor.create_started_stream(StreamType.POINT_CLOUD, None, _point_cloud_callback_func, params)
    print(f"{type(stream)} initialization finished")
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
