from sys_path import *
from api.library_loader import *
from api.errors import ErrorCode, Error
from api.inu_sensor import InuSensor, ConnectionState, create_started_sensor
from api.image_frame import ImageFrame
from api.image_stream import ImageStream
from api.depth_stream import DepthStream
from api.shared import StreamType
import time
import threading
import cv2
import numpy as np
import os
from datetime import datetime

class DualCameraController:
    def __init__(self, camera1_params=None, camera2_params=None):
        """
        初始化双相机控制器
        
        Args:
            camera1_params: 相机1参数 (service_id, ip_address)
            camera2_params: 相机2参数 (service_id, ip_address)
        """
        self.camera1_params = camera1_params or {}
        self.camera2_params = camera2_params or {}
        
        # 创建保存目录
        self.create_save_directories()
        
        # 帧计数器
        self.frame_counter_cam1 = 0
        self.frame_counter_cam2 = 0
        
        
        # 传感器对象
        self.sensor1 = None
        self.sensor2 = None
        
        # 图像流对象
        self.stream1 = None
        self.stream2 = None
        
        # 运行标志
        self.running = False
        
    def create_save_directories(self):
        """创建保存图像的目录"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.save_dir = f"dual_camera_images_{timestamp}"
        self.cam1_dir = os.path.join(self.save_dir, "camera1")
        self.cam2_dir = os.path.join(self.save_dir, "camera2")
        
        os.makedirs(self.cam1_dir, exist_ok=True)
        os.makedirs(self.cam2_dir, exist_ok=True)
        
        print(f"图像将保存到: {self.save_dir}")
        print(f"相机1目录: {self.cam1_dir}")
        print(f"相机2目录: {self.cam2_dir}")
    
    def _sensor_callback_cam1(self, sensor: InuSensor, connection_state: ConnectionState, error: Error) -> None:
        print(f'相机1传感器回调 ConnectionState={connection_state} Error={error}')
    
    def _sensor_callback_cam2(self, sensor: InuSensor, connection_state: ConnectionState, error: Error) -> None:
        print(f'相机2传感器回调 ConnectionState={connection_state} Error={error}')
    
    def _print_frame_cam1(self, frame: ImageFrame) -> None:
        """处理相机1的帧"""
        if frame is None:
            print('相机1: 无效帧')
            return

        current_counter = self.frame_counter_cam1
        self.frame_counter_cam1 += 1
        
        print(f"相机1 - 帧 {current_counter}: 索引 {frame.frame_index}, 格式 {frame.format}, 尺寸 {frame.width}x{frame.height}")
        
        try:
            # 将帧数据转换为图像
            if frame.buffer is not None and len(frame.buffer) > 0:
                # 根据格式确定通道数
                channels = 4  # 默认为4通道
                if frame.bytes_per_pixel == 1:
                    channels = 1
                elif frame.bytes_per_pixel == 3:
                    channels = 3
                
                image_array = np.array(frame.buffer, dtype=np.uint8).reshape(frame.height, frame.width, channels)
                
                # 保存图像
                image_filename = os.path.join(self.cam1_dir, f"cam1_frame_{current_counter:06d}.jpg")
                cv2.imwrite(image_filename, image_array)
                print(f"相机1: 已保存帧 {current_counter} 到 {image_filename}")
                
                # 显示帧信息
                self.display_frame_info("Camera1", current_counter, frame)
        except Exception as e:
            print(f"相机1: 处理帧时出错: {e}")
    
    def _print_frame_cam2(self, frame: ImageFrame) -> None:
        """处理相机2的帧"""
        if frame is None:
            print('相机2: 无效帧')
            return
        

        current_counter = self.frame_counter_cam2
        self.frame_counter_cam2 += 1
        
        print(f"相机2 - 帧 {current_counter}: 索引 {frame.frame_index}, 格式 {frame.format}, 尺寸 {frame.width}x{frame.height}")
        
        try:
            # 将帧数据转换为图像
            if frame.buffer is not None and len(frame.buffer) > 0:
                # 根据格式确定通道数
                channels = 4  # 默认为4通道
                if frame.bytes_per_pixel == 1:
                    channels = 1
                elif frame.bytes_per_pixel == 3:
                    channels = 3
                
                image_array = np.array(frame.buffer, dtype=np.uint8).reshape(frame.height, frame.width, channels)
                
                # 保存图像
                image_filename = os.path.join(self.cam2_dir, f"cam2_frame_{current_counter:06d}.jpg")
                cv2.imwrite(image_filename, image_array)
                print(f"相机2: 已保存帧 {current_counter} 到 {image_filename}")
                
                # 显示帧信息
                self.display_frame_info("Camera2", current_counter, frame)
        except Exception as e:
            print(f"相机2: 处理帧时出错: {e}")
    
    def display_frame_info(self, camera_name, frame_counter, frame):
        """显示帧信息"""
        print(f"{camera_name} 帧 {frame_counter} 信息:")
        print(f"  时间戳: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())}")
        print(f"  帧索引: {frame.frame_index}")
        print(f"  分辨率: {frame.width} x {frame.height}")
        print(f"  格式: {frame.format}")
        print(f"  每像素字节数: {frame.bytes_per_pixel}")
        print("-" * 50)
    
    def _general_camera_callback_cam1(self, stream: ImageStream, frame: ImageFrame, error: Error) -> None:
        """相机1的图像流回调"""
        print("================_general_camera_callback_cam1===============")
        if error is None:
            print(f'相机1: 未定义错误')
        elif error.code != ErrorCode.STATUS_OK:
            print(f'相机1 回调错误 = {error.code} {error.description}')
        else:
            self._print_frame_cam1(frame)
    
    def _general_camera_callback_cam2(self, stream: ImageStream, frame: ImageFrame, error: Error) -> None:
        """相机2的图像流回调"""
        print("================_general_camera_callback_cam2===============")
        if error is None:
            print(f'相机2: 未定义错误')
        elif error.code != ErrorCode.STATUS_OK:
            print(f'相机2 回调错误 = {error.code} {error.description}')
        else:
            self._print_frame_cam2(frame)
    
    def initialize_cameras(self):
        """初始化两个相机"""
        print("开始初始化双相机系统...")
        
        try:
            # 初始化相机1
            print("\n初始化相机1...")
            self.sensor1 = InuSensor()
            hw_info1, dpe_params1 = self.sensor1.init()
            channels_size1 = self.sensor1.start(None, dpe_params1)
            self.sensor1.register = self._sensor_callback_cam1
            print(f"相机1初始化完成. 硬件信息: {hw_info1}")
            
            # 初始化相机2
            print("\n初始化相机2...")
            self.sensor2 = InuSensor()
            hw_info2, dpe_params2 = self.sensor2.init()
            channels_size2 = self.sensor2.start(None, dpe_params2)
            self.sensor2.register = self._sensor_callback_cam2
            print(f"相机2初始化完成. 硬件信息: {hw_info2}")
            
            # 创建并启动图像流
            print("\n创建图像流...")

            self.stream1 = self.sensor1.create_stream(StreamType.GENERAL_CAMERA)
            self.stream2 = self.sensor2.create_stream(StreamType.GENERAL_CAMERA)

            self.stream2.init()
            self.stream2.start()
            self.stream1.init()
            self.stream1.start()

            self.stream2.register = self._general_camera_callback_cam2
            self.stream1.register = self._general_camera_callback_cam1
            
            print("双相机系统初始化完成!")
            self.running = True
            
        except Exception as e:
            print(f"初始化双相机系统时出错: {e}")
            import traceback
            traceback.print_exc()
            self.cleanup()
            raise
    
    def start_capture(self, duration_seconds=10):
        """开始捕获图像"""
        print(f"\n开始双相机同步捕获，持续时间: {duration_seconds}秒...")
        
        start_time = time.time()
        frame_pairs = 0
        
        try:
            while self.running and (time.time() - start_time) < duration_seconds:
                # 可以在这里添加同步逻辑，比如同时触发两个相机
                # 当前使用回调模式，所以不需要主动拉取帧
                
                # 显示当前状态
                elapsed = time.time() - start_time
                print(f"\r运行时间: {elapsed:.1f}秒 | "
                      f"相机1帧数: {self.frame_counter_cam1} | "
                      f"相机2帧数: {self.frame_counter_cam2} | "
                      f"帧对: {min(self.frame_counter_cam1, self.frame_counter_cam2)}", end="")
                
                # # 检查是否同时收到两个相机的帧
                # with self.lock_cam2:
                #     with self.lock_cam1:
                #         if self.frame_counter_cam1 > 0 and self.frame_counter_cam2 > 0:
                #             current_pairs = min(self.frame_counter_cam1, self.frame_counter_cam2)
                #             if current_pairs > frame_pairs:
                #                 frame_pairs = current_pairs
                #                 print(f"\n已同步捕获 {frame_pairs} 对帧")
                
                time.sleep(0.1)  # 短暂休眠，避免占用过多CPU
                
                # 检查键盘中断
                try:
                    pass
                except KeyboardInterrupt:
                    print("\n用户中断捕获")
                    break
        
        except Exception as e:
            print(f"\n捕获过程中出错: {e}")
        
        finally:
            print(f"\n\n捕获完成!")
            print(f"相机1总帧数: {self.frame_counter_cam1}")
            print(f"相机2总帧数: {self.frame_counter_cam2}")
            print(f"同步帧对数: {min(self.frame_counter_cam1, self.frame_counter_cam2)}")
            print(f"图像保存在: {self.save_dir}")
    
    def cleanup(self):
        """清理资源"""
        print("\n正在清理资源...")
        self.running = False
        
        try:
            if self.stream1:
                self.stream1.stop()
                self.stream1.terminate()
                print("相机1流已停止")
        except:
            pass
        
        try:
            if self.stream2:
                self.stream2.stop()
                self.stream2.terminate()
                print("相机2流已停止")
        except:
            pass
        
        try:
            if self.sensor1:
                self.sensor1.register = None
                self.sensor1.stop()
                self.sensor1.terminate()
                print("相机1传感器已停止")
        except:
            pass
        
        try:
            if self.sensor2:
                self.sensor2.register = None
                self.sensor2.stop()
                self.sensor2.terminate()
                print("相机2传感器已停止")
        except:
            pass
        
        print("资源清理完成")


rgb_channel = 9
depth_channel = 5  # 5, 3

rgb_stream = None
depth_stream = None

rgb_idx = 0
depth_idx = 0

rgb_frame = None
depth_frame = None

frame_counter_cam1 = 0
frame_counter_cam2 = 0

# Condition object for synchronization (similar to std::condition_variable)
rgb_condition = threading.Condition()
depth_condition = threading.Condition()

def _sensor_callback_func(sensor: InuSensor, connection_state: ConnectionState, error: Error) -> None:
    print(f'Sensor callback ConnectionState={connection_state} Error={error}')

def _print_frame_cam1(self, frame: ImageFrame) -> None:
    global frame_counter_cam1
    """处理相机1的帧"""
    if frame is None:
        print('相机1: 无效帧')
        return

    current_counter = self.frame_counter_cam1
    self.frame_counter_cam1 += 1
    
    print(f"相机1 - 帧 {current_counter}: 索引 {frame.frame_index}, 格式 {frame.format}, 尺寸 {frame.width}x{frame.height}")
    
    try:
        # 将帧数据转换为图像
        if frame.buffer is not None and len(frame.buffer) > 0:
            # 根据格式确定通道数
            channels = 4  # 默认为4通道
            if frame.bytes_per_pixel == 1:
                channels = 1
            elif frame.bytes_per_pixel == 3:
                channels = 3
            
            image_array = np.array(frame.buffer, dtype=np.uint8).reshape(frame.height, frame.width, channels)
            
            # 保存图像
            image_filename = os.path.join(cam1_dir, f"cam1_frame_{current_counter:06d}.jpg")
            cv2.imwrite(image_filename, image_array)
            print(f"相机1: 已保存帧 {current_counter} 到 {image_filename}")
            
    except Exception as e:
        print(f"相机1: 处理帧时出错: {e}")
    
def _print_frame_cam2(frame: ImageFrame) -> None:
    """处理相机2的帧"""
    global frame_counter_cam2
    if frame is None:
        print('相机2: 无效帧')
        return
    
    current_counter = frame_counter_cam2
    frame_counter_cam2 += 1
    
    print(f"相机2 - 帧 {current_counter}: 索引 {frame.frame_index}, 格式 {frame.format}, 尺寸 {frame.width}x{frame.height}")
    
    try:
        # 将帧数据转换为图像
        if frame.buffer is not None and len(frame.buffer) > 0:
            # 根据格式确定通道数
            channels = 4  # 默认为4通道
            if frame.bytes_per_pixel == 1:
                channels = 1
            elif frame.bytes_per_pixel == 3:
                channels = 3
            
            image_array = np.array(frame.buffer, dtype=np.uint8).reshape(frame.height, frame.width, channels)
            
            # 保存图像
            image_filename = os.path.join(cam2_dir, f"cam2_frame_{current_counter:06d}.jpg")
            cv2.imwrite(image_filename, image_array)
            print(f"相机2: 已保存帧 {current_counter} 到 {image_filename}")
            
    except Exception as e:
        print(f"相机2: 处理帧时出错: {e}")



# 主程序
if __name__ == "__main__":
    # 配置两个相机的参数
    # 根据实际情况修改这些参数
    sensor, channels_size, hw_information = create_started_sensor(_sensor_callback_func)
    print(f"Sensor initialization finished")
    depth_stream = sensor.create_started_stream(StreamType.DEPTH, depth_channel, _stream_callback_func)
    print(f"Depth stream initialization finished. Channel: {depth_stream.channel_id}")
    rgb_stream = sensor.create_started_stream(StreamType.GENERAL_CAMERA, rgb_channel, _stream_callback_func)
    print(f"Image stream initialization finished. Channel: {rgb_stream.channel_id}")
    
    # 如果两个相机都是本地USB相机，可以尝试不同的service_id
    # 或者都留空，如果SDK能自动识别多个相机
    
    controller = DualCameraController(camera1_config, camera2_config)
    
    try:
        # 初始化相机
        controller.initialize_cameras()
        
        # 开始捕获，持续30秒
        controller.start_capture(duration_seconds=30)
        
    except Exception as e:
        print(f"主程序出错: {e}")
        
    finally:
        # 确保清理资源
        controller.cleanup()