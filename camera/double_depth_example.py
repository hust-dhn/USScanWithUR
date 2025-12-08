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
import threading

frame_counter = 0



# #########################################################  main #####################################################
def _sensor_callback_func(self, sensor: InuSensor, connection_state: ConnectionState, error: Error) -> None:
    print(f'Sensor callback ConnectionState={connection_state} Error={error}')

def _print_frame(frame: ImageFrame) -> None:
    global frame_counter  # Use the global counter to save each frame with a unique name
    if frame is None:
        print('Invalid Depth frame')
    else:
        print(f"\t------------------------------------- Depth {frame.frame_index} ---------------------------------")
        print(f"{type(frame)} : Format {frame.format}, Height {frame.height}, Width{frame.width}, "
            f"BytesPerPixel {frame.bytes_per_pixel}")
        print("\t------------------------------------------------------------------------------------------")
        
        # Access the image data from 'image_frame' attribute
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

def capture_from_camera(camera_id: int, sensor: InuSensor, stream_type: StreamType, callback_func) -> None:
    print(f"Start camera {camera_id} test application")
    
    hw_information = sensor.init()
    channels_size, hw_information = sensor.start(hw_information)
    sensor.register = _sensor_callback_func
    print(f"Sensor {camera_id} initialization finished")
    
    stream = sensor.create_stream(stream_type)
    stream.init()
    stream.start()
    stream.register = callback_func
    print(f"Camera {camera_id} stream initialization finished")
    
    for i in range(0, 5000):
        try:
            _print_frame(stream.frame)
            print(f"Camera {camera_id}, iteration {i}")
            time.sleep(1)
        except KeyboardInterrupt:
            break
    
    print(f"Finish camera {camera_id} test application")
    stream.stop()
    stream.terminate()
    print(f"Camera {camera_id} {type(stream)} Stopped")
    sensor.register = None
    sensor.stop()
    sensor.terminate()
    print(f'Camera {camera_id} Sensor stopped')

if __name__ == "__main__":
    # 创建两个相机实例
    sensor1 = InuSensor()
    sensor2 = InuSensor()

    # 创建两个线程来并行运行相机捕获任务
    thread1 = threading.Thread(target=capture_from_camera, args=(1, sensor1, StreamType.GENERAL_CAMERA, _general_camera_callback_func))
    thread2 = threading.Thread(target=capture_from_camera, args=(2, sensor2, StreamType.GENERAL_CAMERA, _general_camera_callback_func))
    
    # 启动线程
    thread1.start()
    thread2.start()
    
    # 等待两个线程完成
    thread1.join()
    thread2.join()
