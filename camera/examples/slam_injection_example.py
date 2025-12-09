from sys_path import *
from api.library_loader import *
from api.errors import ErrorCode, Error
from api.inu_sensor import InuSensor,  InjectionType, ConnectionState, DeviceParams, create_sensor
from api.image_frame import ImageFrame
from api.slam import SlamFrame, SlamStream
from api.shared import StreamType, Point2Dim
import time
import numpy as np
from PIL import Image
if sys.platform == 'linux':
    import signal


def load_image(image: str) -> np.array:
    image = Image.open(image)  # Open the TIFF image file
    if image is None:
        raise Exception("Sorry, Failed open in load_image.")
    w, h = image.size  # Get image width and height in pixels
    mode = image.mode  # Get the image mode (e.g., 'RGB', 'L', 'RGBA', etc.)
    # Calculate the byte per pixel (BPP) based on the image mode
    bp = 0
    if mode == 'I;16':
        bp = 2
    # elif mode == 'L':
    #     bp = 1  # 8-bit grayscale image
    # elif mode == 'P':
    #     bp = 1  # 8-bit palette-based image
    elif mode == 'RGB':
        bp = 3  # 24-bit RGB image (8 bits per channel)
    elif mode == 'RGBA':
        bp = 4  # 32-bit RGBA image (8 bits per channel + alpha channel)
    else:
        raise Exception("Sorry, load_image unsupported mode.")
    total_bytes = w * h * bp
    if total_bytes == 0:
        raise Exception("Sorry, load_image total_bytes == 0.")
    pixel_data = image.tobytes()  # Get the pixel data as a bytes object
    # Create a copy of the pixel data as a writable NumPy array
    pixel_arr = np.copy(np.frombuffer(pixel_data, dtype=np.uint8))
    # Reshape the array to match the image dimensions
    pixel_arr = pixel_arr.reshape(h, w, bp)
    image.close()  # Close the image file when you're done
    print(f"load_image  Width: {w} Height: {h} BPP: {bp} ")
    return pixel_arr


# ################################################### main #############################################################
if __name__ == "__main__":
    def _sensor_callback_func(self, sensor: InuSensor, connection_state: ConnectionState, error: Error) -> None:
        print(f'Sensor callback ConnectionState={connection_state} Error={error}')


    def _print_frame(frame: SlamFrame) -> None:
        if frame is None:
            print('Invalid SlamFrame frame')
        else:
            print(f"\t---------- {frame} frame index {frame.frame_index} time {frame.time_stamp} ----------------")
            print(f"\t State {frame.state} InternalState {frame.internal_state}")
            print("\t Pose4x4BodyToWorld")
            for b2w in frame.pose4x4body2world:
                print("\t\t{}".format(b2w))
            print("\t Pose4x4WorldToBody")
            for w2b in frame.pose4x4world2body:
                print(f"\t\t{w2b}")

    print(f"Start test application")
    sensor = create_sensor()
    device_params = DeviceParams()
    device_params.fps = 5
    device_params.injection_type = InjectionType.IDVE
    hw_information, dpe = sensor.init(device_params)
    tracking_channel_id = 2
    map_string_injector_params = hw_information.get_injectors(tracking_channel_id)
    keys = list(map_string_injector_params)
    inject_streamer_name = keys[0]
    inject_resolution = Point2Dim(int(1280 / 2), 480)
    sensor.set_inject_resolution(inject_streamer_name, inject_resolution)
    channels_size = sensor.start()
    sensor.register = _sensor_callback_func
    print(f"Sensor initialization finished")
    slam = sensor.create_started_stream(StreamType.SLAM)
    print("Slam stream initialization finished")
    injection = sensor.create_started_stream(StreamType.INJECTION, inject_streamer_name)
    print("Injection stream initialization finished")
    if sys.platform == 'win32':
        image_path = r"C:\Program Files\Inuitive\InuDev\Injection\Tracking\Injection_FisheyeImage.tif"
    elif sys.platform == 'linux':
        image_path = r"/opt/Inuitive/InuDev/Injection/Tracking/Injection_FisheyeImage.tif"
    else:
        raise Exception("This platform doesn't  supported yet.")
    pixel_array = load_image(image_path)
    for i in range(0, 5000):
        try:
            # _print_frame(injection.inject_file_to_stream(StreamType.TRACKING, image_path))
            start = time.time()
            frame = injection.inject_data_to_stream(StreamType.SLAM, pixel_array)
            _print_frame(frame)
            stop = time.time()
            print(f"injection.inject_data_to_stream tacked  {stop - start}")
            print(f"iteration {i}")
            time.sleep(1)
        except KeyboardInterrupt:
            break
    slam.terminate()
    print(f"Slam stream Stopped")
    injection.terminate()
    print(f"Injection stream Stopped")
    sensor.terminate()
    print(f'Sensor stopped')
    if sys.platform == 'win32':
        os._exit(0)
    elif sys.platform == 'linux':
        os.kill(os.getpid(), signal.SIGINT)
    else:
        sys.exit(0)
