from sys_path import *
from api.library_loader import *
from api.errors import ErrorCode, Error
from api.inu_sensor import InuSensor, ConnectionState, DeviceParams, create_started_sensor
from api.cnn import CnnStream, CnnFrame
from api.cnn_defs import CnnParams
from api.shared import StreamType
from api.image_stream import ImageStream
from api.image_frame import ImageFrame
import time
if sys.platform == 'linux':
    import signal

# ################################################### main #############################################################
if __name__ == "__main__":
    def _sensor_callback_func(self, sensor: InuSensor, connection_state: ConnectionState, error: Error) -> None:
        print(f'Sensor callback ConnectionState={connection_state} Error={error}')


    def _print_image_frame(frame: ImageFrame) -> None:
        print(
            f'ImageFrame Format {frame.format}, Height {frame.height}, Width {frame.width}), '
            f'BytesPerPixel {frame.bytes_per_pixel}')


    def _print_cnn_frame(frame: CnnFrame) -> None:
        hdr = frame.frame_header
        print(
            f"\t EngineType {frame.engine_type} TotalResultSize {hdr.total_result_size} NumOfTails {hdr.num_of_tails} "
            f"NetworkId {hdr.network_id} FrameId {hdr.frame_id}")
        tail_headers = hdr.tail_header
        for th in tail_headers:
            print(f"\t\t TailHeader # {th}")
            print(
                f"\t\t ElementSize {th.ElementSize} ElementCount {th.ElementCount} Width {th.Width} Height {th.Height} "
                f"Inputs {th.Inputs}")
            sf = th.CnnFields.SynopsysFields
            print(f"\t\t SynopsysFields : Scale {sf.Scale} AllocCount {sf.AllocCount} ValidCount {sf.ValidCount} "
                  f"BboxScale {sf.BboxScale} ConfidenceScale {sf.ConfidenceScale}")
            print(f"\t\t CevaFields : FracBits {th.CnnFields.CevaFields.FracBits}")
        offsets_to_blob = hdr.offset_to_blob
        blob = frame.buffer
        for otb in offsets_to_blob:
            print(f"\t\t OffsetsToBlob : {otb}")
            print(f"\t\t Blob :  {blob + otb}")

    def _print_frame(frame: CnnFrame) -> None:
        if frame is None:
            raise Exception(f"Invalids frame.")
        print(f"print_frame")
        print(f"\t---------- {type(frame)} index {frame.frame_index} time {frame.time_stamp} ------------")
        if type(frame) == ImageFrame:
            _print_image_frame(frame)
        if type(frame) == CnnFrame:
            _print_cnn_frame(frame)
        else:
            raise Exception(f"This frame type {type(frame)}  doesn't supported.")


    def _cnn_callback_func(stream: CnnStream, frame: CnnFrame, error: Error) -> None:
        if error is None:
            print(f'Undefined error in {type(stream)}')
        elif error.code != ErrorCode.STATUS_OK:
            print(f'{type(stream)} stream callback Error = {error.code} {error.description}')
        else:
            print(f'{type(stream)} stream')
            print("cnn_callback_func before print")
            _print_frame(frame)


    def _get_camera_callback_func(stream: ImageStream, frame: ImageFrame, error: Error) -> None:
        if error is None:
            print(f'Undefined error in {type(stream)} gen camera')
        elif error.code != ErrorCode.STATUS_OK:
            print(f'{type(stream)} stream callback Error = {error.code} {error.description}')
        else:
            print(f'Inj {type(stream)} stream')
            _print_frame(frame)

    print(f"Start test application")
    cnn_param = CnnParams()
    cnn_param.network_id = str(0)
    if sys.platform == 'win32':
        # cnn_param.network_file_name = "C:/Program Files/Inuitive/InuDev/config/AI4100/cnn_bins/nn_bin_yolov7_tiny.bin"
        cnn_param.network_file_name = "C:/Program Files/Inuitive/InuDev/config/AI4000/cnn_bins/nn_bin_yolov7_tiny.bin"
    elif sys.platform == 'linux':
        # cnn_param.network_file_name = "/opt/Inuitive/InuDev/config/AI4100/cnn_bins/nn_bin_yolov7_tiny.bin"
        cnn_param.network_file_name = "/opt/Inuitive/InuDev/config/AI4000/cnn_bins/nn_bin_yolov7_tiny.bin"
    else:
        raise Exception("This platform doesn't  supported yet.")
    device_params = DeviceParams()
    device_params.fps = 5
    sensor, channels_size, hw_information = create_started_sensor(_sensor_callback_func, device_params, cnn_param)
    print(f"{type(sensor)} initialization finished")
    gen_camera_stream = sensor.create_started_stream(StreamType.GENERAL_CAMERA, 4, _get_camera_callback_func)
    print(f"{type(gen_camera_stream)} gen camera {4} initialization finished")
    cnn_stream = sensor.create_started_stream(StreamType.CNN, "Sout_cnn_0",
                                              _cnn_callback_func, cnn_param.network_id,
                                              "cnn_sensor_0")
    print(f"{type(cnn_stream)} cnn {'Sout_cnn_0'} initialization finished")
    for i in range(0, 5000):
        try:
            # _print_frame(cnn_stream.frame)
            print(f"iteration {i}")
            time.sleep(1)
        except KeyboardInterrupt:
            break
    print(f"Finish test application")
    gen_camera_stream.terminate()
    print(f"{type(gen_camera_stream)} Stopped")
    cnn_stream.terminate()
    print(f"{type(cnn_stream)} Stopped")
    sensor.terminate()
    print(f'Sensor stopped')
    if sys.platform == 'win32':
        os._exit(0)
    elif sys.platform == 'linux':
        os.kill(os.getpid(), signal.SIGINT)
    else:
        sys.exit(0)
