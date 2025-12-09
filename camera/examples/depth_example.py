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
    def _sensor_callback_func(sensor: InuSensor, connection_state: ConnectionState, error: Error) -> None:
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
    
    try:
        # 关键修改1: init() 返回的是一个元组 (HwInfo, VectorDpeParams)
        # 需要用两个变量接收
        hw_information, dpe_params = sensor.init()
        print(f"初始化成功。hw_information 类型: {type(hw_information)}")
        
        # 关键修改2: start() 期望的参数是字典 (或None) 和 dpe_params (或None)
        # 这里我们先使用默认参数，传 None, dpe_params
        channels_size = sensor.start(None, dpe_params)
        
        print(f"hw_information: {hw_information}")
        print(f"channels_size 类型: {type(channels_size)}")
        
        # 如果channels_size是MapUintPoint类型，可能需要遍历打印
        if hasattr(channels_size, 'items'):
            print("各通道的图像尺寸:")
            for channel_id, size in channels_size.items():
                print(f"  通道 {channel_id}: {size}")
        
        # 注册传感器状态回调
        sensor.register = _sensor_callback_func
        print(f"Sensor initialization finished")
        
        # 创建并启动图像流
        # stream = sensor.create_stream(StreamType.DEPTH)
        stream = sensor.create_stream(StreamType.GENERAL_CAMERA)
        stream.init()
        stream.start()
        
        # 注册数据流回调
        # stream.register = _depth_callback_func
        # print("Depth stream initialization finished")
        stream.register = _general_camera_callback_func
        print("General Camera stream initialization finished")
        
        # 主循环 - 主动拉取帧数据
        for i in range(0, 5000):
            try:
                # 注意：这里直接访问stream.frame可能返回None，特别是如果使用回调模式
                current_frame = stream.frame
                if current_frame:
                    _print_frame(current_frame)
                else:
                    print(f"iteration {i}: 未获取到帧")
                print(f"iteration {i}")
                time.sleep(1)
            except KeyboardInterrupt:
                print("用户中断")
                break
                
    except Exception as e:
        print(f"运行过程中发生错误: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # 确保资源被释放
        print(f"Finish test application")
        try:
            if 'stream' in locals():
                stream.stop()
                stream.terminate()
                print(f"{type(stream)} Stopped")
        except:
            pass
            
        try:
            if 'sensor' in locals():
                sensor.register = None
                sensor.stop()
                sensor.terminate()
                print(f'Sensor stopped')
        except:
            pass