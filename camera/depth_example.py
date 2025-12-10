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

frame_counter1 = 0
frame_counter2 = 0

if __name__ == "__main__":
    def _sensor_callback_func1(sensor: InuSensor, connection_state: ConnectionState, error: Error) -> None:
        print(f'Sensor callback ConnectionState1={connection_state} Error={error}')
    def _sensor_callback_func2(sensor: InuSensor, connection_state: ConnectionState, error: Error) -> None:
        print(f'Sensor callback ConnectionState2={connection_state} Error={error}')
    def _print_frame1(frame: ImageFrame) -> None:
        global frame_counter1  # Use the global counter to save each frame with a unique name
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
                image_filename = f"{frame_counter1}.jpg"
                cv2.imwrite(image_filename, depth_image)  # Save the image as a .jpg file
                print(f"Saved frame {frame_counter1} as {image_filename}")
            
            # Increment the frame counter for the next image
            frame_counter1 += 1
    def _print_frame2(frame: ImageFrame) -> None:
        global frame_counter2  # Use the global counter to save each frame with a unique name
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
                image_filename = f"{frame_counter2}.jpg"
                cv2.imwrite(image_filename, depth_image)  # Save the image as a .jpg file
                print(f"Saved frame {frame_counter2} as {image_filename}")
            
            # Increment the frame counter for the next image
            frame_counter2 += 1
    
    #def _depth_callback_func(stream: DepthStream, frame: ImageFrame, error: Error) -> None:
    #    if error is None:
    #        print(f'Undefined error in {type(stream)}')
    #    elif error.code != ErrorCode.STATUS_OK:
    #        print(f'{type(stream)} callback Error = {error.code} {error.description}')
    #    else:
    #        _print_frame(frame)

    def _general_camera_callback_func1(stream: ImageStream, frame: ImageFrame, error: Error) -> None:
        if error is None:
            print(f'Undefined error in {type(stream)}')
        elif error.code != ErrorCode.STATUS_OK:
            print(f'{type(stream)} callback Error = {error.code} {error.description}')
        else:
            _print_frame1(frame)
    def _general_camera_callback_func2(stream: ImageStream, frame: ImageFrame, error: Error) -> None:
        if error is None:
            print(f'Undefined error in {type(stream)}')
        elif error.code != ErrorCode.STATUS_OK:
            print(f'{type(stream)} callback Error = {error.code} {error.description}')
        else:
            _print_frame2(frame)

    print(f"Start test application")
    sensor1 = InuSensor('1','')
    sensor2 = InuSensor('2','')

    try:
        # 关键修改1: init() 返回的是一个元组 (HwInfo, VectorDpeParams)
        # 需要用两个变量接收
        hw_information1, dpe_params1 = sensor1.init()
        hw_information2, dpe_params2 = sensor2.init()
        print(f"camera1初始化成功。hw_information 类型: {type(hw_information1)}")
        print(f"camera2初始化成功。hw_information 类型: {type(hw_information2)}")
        # 关键修改2: start() 期望的参数是字典 (或None) 和 dpe_params (或None)
        # 这里我们先使用默认参数，传 None, dpe_params
        channels_size1 = sensor1.start(None, dpe_params1)
        channels_size2 = sensor2.start(None, dpe_params2)
        print(f"hw_information1: {hw_information1}")
        print(f"channels_size1 类型: {type(channels_size1)}")
        print(f"hw_information2: {hw_information2}")
        print(f"channels_size2 类型: {type(channels_size2)}")  

        # 如果channels_size是MapUintPoint类型，可能需要遍历打印
        if hasattr(channels_size1, 'items'):
            print("camera1各通道的图像尺寸:")
            for channel_id, size in channels_size1.items():
                print(f"  通道 {channel_id}: {size}")
        if hasattr(channels_size2, 'items'):
            print("camera2各通道的图像尺寸:")
            for channel_id, size in channels_size2.items():
                print(f"  通道 {channel_id}: {size}")

        # 注册传感器状态回调
        sensor1.register = _sensor_callback_func1
        print(f"camera1_Sensor1 initialization finished")
        sensor2.register = _sensor_callback_func2
        print(f"camera2_Sensor2 initialization finished")

        # 创建并启动图像流
        stream1 = sensor1.create_stream(StreamType.GENERAL_CAMERA)
        stream2 = sensor2.create_stream(StreamType.GENERAL_CAMERA)
        stream1.init()
        stream2.init()
        stream1.start()
        stream2.start()

        # 注册数据流回调
        stream1.register = _general_camera_callback_func1
        print("General Camera1 stream initialization finished")
        stream2.register = _general_camera_callback_func2
        print("General Camera2 stream initialization finished")

        for i in range(0, 5000):
            try:
                # 注意：这里直接访问stream.frame可能返回None，特别是如果使用回调模式
                current_frame1 = stream1.frame
                current_frame2 = stream2.frame
                if current_frame1:
                    _print_frame1(current_frame1)
                    _print_frame2(current_frame2)
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
                stream1.stop()
                stream2.stop()
                stream1.terminate()
                stream2.terminate()
                print(f"{type(stream1)} Stopped")
                print(f"{type(stream2)} Stopped")
        except:
            pass
            
        try:
            if 'sensor' in locals():
                sensor1.register = None
                sensor2.register = None
                sensor1.stop()
                sensor2.stop()
                sensor1.terminate()
                sensor2.terminate()
                print(f'Sensor stopped')
        except:
            pass