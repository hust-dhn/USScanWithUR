from sys_path import *
from api.library_loader import *
from api.errors import ErrorCode, Error
from api.inu_sensor import InuSensor, ConnectionState
from api.image_frame import ImageFrame
from api.image_stream import ImageStream
from api.depth_stream import DepthStream
from api.shared import StreamType
import time

import cv2
import numpy as np
frame_counter = 0

if __name__ == "__main__":
    def _sensor_callback_func(self, sensor: InuSensor, connection_state: ConnectionState, error: Error) -> None:
        print(f'Sensor callback ConnectionState={connection_state} Error={error}')

    def _print_frame(frame: ImageFrame) -> None:
        global frame_counter  # Use the global counter to save each frame with a unique name
        if frame is None:
            print('Invalid RGB frame')
        else:
            print(f"\t------------------------------------- Depth {frame.frame_index} ---------------------------------")
            print(f"{type(frame)} : Format {frame.format}, Height {frame.height}, Width{frame.width}, "
                f"BytesPerPixel {frame.bytes_per_pixel}")
            print("\t------------------------------------------------------------------------------------------")

            depth_image = frame.image_frame  # Assuming 'image_frame' contains the image data
            
            print(f"depth_image.shape: {depth_image}")

            if depth_image is not None:
                # Ensure that the image is in the correct format (e.g., numpy array)
                depth_image = np.array(frame.buffer, dtype=np.uint8).reshape(frame.height, frame.width, 4)

                # Save the frame as an image using OpenCV
                image_filename = f"{frame_counter}.jpg"
                cv2.imwrite(image_filename, depth_image)  # Save the image as a .jpg file
                print(f"Saved frame {frame_counter} as {image_filename}")
            
            # Increment the frame counter for the next image
            frame_counter += 1

    def _depth_callback_func(stream: DepthStream, frame: ImageFrame, error: Error) -> None:
        if error is None:
            print(f'Undefined error in {type(stream)}')
        elif error.code != ErrorCode.STATUS_OK:
            print(f'{type(stream)} callback Error = {error.code} {error.description}')
        else:
            _print_frame(frame)

    def _general_camera_callback_func(stream: ImageStream, frame: ImageFrame, error: Error) -> None:
        if error is None:
            print(f'Undefined error in {type(stream)}')
        elif error.code != ErrorCode.STATUS_OK:
            print(f'{type(stream)} callback Error = {error.code} {error.description}')
        else:
            _print_frame(frame)


    print(f"Start test application")
    sensor = InuSensor()
    hw_information = sensor.init()
    channels_size, hw_information = sensor.start(hw_information)
    print(f"hw_information: {hw_information}")
    print(f"channels_size: {channels_size}")
    sensor.register = _sensor_callback_func
    print(f"Sensor initialization finished")
    # stream = sensor.create_stream(StreamType.DEPTH)
    stream = sensor.create_stream(StreamType.GENERAL_CAMERA)
    stream.init()
    stream.start()
    # stream.register = _depth_callback_func
    # print("Depth stream initialization finished")
    stream.register = _general_camera_callback_func
    print("General Camera stream initialization finished")
    for i in range(0, 5000):
        try:
            _print_frame(stream.frame)
            print(f"iteration {i}")
            time.sleep(1)
        except KeyboardInterrupt:
            break
    print(f"Finish test application")
    stream.stop()
    stream.terminate()
    print(f"{type(stream)} Stopped")
    sensor.register = None
    sensor.stop()
    sensor.terminate()
    print(f'Sensor stopped')
