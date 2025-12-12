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
from function.clear_files import delete_jpg_files


class DualCamera:
    """双相机封装类，用于超声波扫描系统的图像采集"""
    
    def __init__(self, lc_imgs_path="camera/lc_imgs/", rc_imgs_path="camera/rc_imgs/", 
                 clear_on_init=True):
        """
        初始化双相机类
        
        :param lc_imgs_path: 左相机图像保存路径
        :param rc_imgs_path: 右相机图像保存路径
        :param clear_on_init: 初始化时是否清空已有图像文件
        """
        self.lc_imgs_path = lc_imgs_path
        self.rc_imgs_path = rc_imgs_path
        self.clear_on_init = clear_on_init
        
        # 相机对象
        self.sensor1 = None
        self.sensor2 = None
        self.stream1 = None
        self.stream2 = None
        
        # 帧计数器
        self.frame_counter1 = 0
        self.frame_counter2 = 0

        self.channels_size1 = None
        self.channels_size2 = None
        
        # 是否初始化
        self.is_initialized = False
    
    def _sensor_callback_func1(self, sensor: InuSensor, connection_state: ConnectionState, error: Error) -> None:
        """相机1传感器回调函数"""
        print(f'[Camera 1] Sensor callback ConnectionState={connection_state} Error={error}')
    
    def _sensor_callback_func2(self, sensor: InuSensor, connection_state: ConnectionState, error: Error) -> None:
        """相机2传感器回调函数"""
        print(f'[Camera 2] Sensor callback ConnectionState={connection_state} Error={error}')
    
    def _save_frame1(self, frame: ImageFrame) -> None:
        """保存左相机帧"""
        if frame is None:
            print('[Camera 1] Invalid frame')
        else:
            print(f"\t[Camera 1] Frame {frame.frame_index} - Format: {frame.format}, Size: {frame.width}x{frame.height}")
            
            if frame.buffer is not None:
                # 将缓冲区转换为图像
                depth_image = np.array(frame.buffer, dtype=np.uint8).reshape(frame.height, frame.width, 4)
                
                # 保存图像
                image_filename = f"{self.lc_imgs_path}{self.frame_counter1}.jpg"
                cv2.imwrite(image_filename, depth_image)
                print(f"[Camera 1] Saved frame {self.frame_counter1} as {image_filename}")
            
            self.frame_counter1 += 1
    
    def _save_frame2(self, frame: ImageFrame) -> None:
        """保存右相机帧"""
        if frame is None:
            print('[Camera 2] Invalid frame')
        else:
            print(f"\t[Camera 2] Frame {frame.frame_index} - Format: {frame.format}, Size: {frame.width}x{frame.height}")
            
            if frame.buffer is not None:
                # 将缓冲区转换为图像
                depth_image = np.array(frame.buffer, dtype=np.uint8).reshape(frame.height, frame.width, 4)
                
                # 保存图像
                image_filename = f"{self.rc_imgs_path}{self.frame_counter2}.jpg"
                cv2.imwrite(image_filename, depth_image)
                print(f"[Camera 2] Saved frame {self.frame_counter2} as {image_filename}")
            
            self.frame_counter2 += 1
    
    def _stream_callback_func1(self, stream: ImageStream, frame: ImageFrame, error: Error) -> None:
        """相机1数据流回调函数"""
        if error is None:
            print(f'[Camera 1 Stream] Undefined error')
        elif error.code != ErrorCode.STATUS_OK:
            print(f'[Camera 1 Stream] Error = {error.code} {error.description}')
        else:
            self._save_frame1(frame)
    
    def _stream_callback_func2(self, stream: ImageStream, frame: ImageFrame, error: Error) -> None:
        """相机2数据流回调函数"""
        if error is None:
            print(f'[Camera 2 Stream] Undefined error')
        elif error.code != ErrorCode.STATUS_OK:
            print(f'[Camera 2 Stream] Error = {error.code} {error.description}')
        else:
            self._save_frame2(frame)
    
    def initialize(self):
        """初始化双相机"""
        try:
            # 清空图像文件
            if self.clear_on_init:
                delete_jpg_files(self.lc_imgs_path)
                delete_jpg_files(self.rc_imgs_path)
                print(f"[Init] Cleared image files from {self.lc_imgs_path} and {self.rc_imgs_path}")
            
            print(f"[Init] Initializing dual cameras...")
            
            # 初始化两个传感器
            self.sensor1 = InuSensor('1', '')
            self.sensor2 = InuSensor('2', '')
            
            # 初始化传感器
            hw_information1, dpe_params1 = self.sensor1.init()
            hw_information2, dpe_params2 = self.sensor2.init()
            print(f"[Init] Sensor 1 initialized: {type(hw_information1)}")
            print(f"[Init] Sensor 2 initialized: {type(hw_information2)}")
            
            # 启动传感器
            self.channels_size1 = self.sensor1.start(None, dpe_params1)
            self.channels_size2 = self.sensor2.start(None, dpe_params2)
                    
            # 注册传感器回调
            self.sensor1.register = self._sensor_callback_func1
            self.sensor2.register = self._sensor_callback_func2
            print(f"[Init] Sensor callbacks registered")
            
            # 创建并启动图像流
            self.stream1 = self.sensor1.create_stream(StreamType.GENERAL_CAMERA)
            self.stream2 = self.sensor2.create_stream(StreamType.GENERAL_CAMERA)
            
            self.stream1.init()
            self.stream2.init()
            self.stream1.start()
            self.stream2.start()
            
            print(f"[Init] Image streams started")
            
            self.is_initialized = True
            print(f"[Init] Dual camera initialization completed successfully")
            return True
        
        except Exception as e:
            print(f"[Error] Failed to initialize dual camera: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def get_frames(self):
        """
        获取当前帧（非阻塞）
        
        :return: (frame1, frame2) 左右相机的当前帧，如果无可用帧则返回None
        """
        if not self.is_initialized:
            print("[Error] Camera not initialized")
            return None, None
        
        try:
            frame1 = self.stream1.frame
            frame2 = self.stream2.frame
            return frame1, frame2
        except Exception as e:
            print(f"[Error] Failed to get frames: {e}")
            return None, None
    
    def _frame_to_image(self, frame: ImageFrame):
        """
        将 ImageFrame 转换为 numpy 图像数组
        
        :param frame: ImageFrame 对象
        :return: numpy 数组（BGR 格式）或 None
        """
        if frame is None or frame.buffer is None:
            return None
        
        try:
            # 将缓冲区转换为 numpy 数组，假设是 RGBA 格式 (4 通道)
            img = np.array(frame.buffer, dtype=np.uint8).reshape(frame.height, frame.width, 4)
            # 转换为 BGR 格式用于 OpenCV 显示
            img_bgr = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
            return img_bgr
        except Exception as e:
            print(f"[Error] Failed to convert frame to image: {e}")
            return None
    
    def display_live_view(self, duration=None, window_width=640, window_height=480):
        """
        实时显示左右相机的画面（类似视频图窗）
        
        :param duration: 显示时长（秒），None 表示持续显示直到按 'q' 键退出
        :param window_width: 窗口宽度（像素）
        :param window_height: 窗口高度（像素）
        :return: 是否成功
        """
        if not self.is_initialized:
            print("[Error] Camera not initialized")
            return False
        
        # 创建两个窗口
        window_name_lc = "Left Camera (LC)"
        window_name_rc = "Right Camera (RC)"
        cv2.namedWindow(window_name_lc, cv2.WINDOW_NORMAL)
        cv2.namedWindow(window_name_rc, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name_lc, window_width, window_height)
        cv2.resizeWindow(window_name_rc, window_width, window_height)
        
        print(f"[Display] Live view started. Press 'q' to quit.")
        if duration:
            print(f"[Display] Will display for {duration} seconds.")
        
        start_time = time.time()
        
        try:
            while True:
                # 检查时长限制
                if duration is not None:
                    elapsed = time.time() - start_time
                    if elapsed >= duration:
                        print(f"[Display] Duration limit reached ({duration}s)")
                        break
                
                # 获取当前帧
                frame1, frame2 = self.get_frames()
                
                if frame1 is not None and frame2 is not None:
                    # 转换为 OpenCV 可显示的格式
                    img_lc = self._frame_to_image(frame1)
                    img_rc = self._frame_to_image(frame2)
                    
                    if img_lc is not None:
                        cv2.imshow(window_name_lc, img_lc)
                    if img_rc is not None:
                        cv2.imshow(window_name_rc, img_rc)
                else:
                    print(f"[Warning] No frames available at {time.time() - start_time:.2f}s")
                
                # 按 'q' 键退出，延迟 1 毫秒
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print(f"[Display] User quit")
                    break
                
                # 小延迟避免 CPU 过载
                time.sleep(0.03)
        
        except KeyboardInterrupt:
            print("[Display] Interrupted by user")
        
        except Exception as e:
            print(f"[Error] Error during display: {e}")
            import traceback
            traceback.print_exc()
        
        finally:
            # 关闭窗口
            cv2.destroyAllWindows()
            print(f"[Display] Live view closed")
        
        return True
    
    def capture_frames(self, num_frames=1, interval=0.1):
        """
        连续获取多帧图像
        
        :param num_frames: 获取的帧数
        :param interval: 帧之间的时间间隔（秒）
        """
        if not self.is_initialized:
            print("[Error] Camera not initialized")
            return False
        
        try:
            for i in range(num_frames):
                frame1, frame2 = self.get_frames()
                
                if frame1 and frame2:
                    self._save_frame1(frame1)
                    self._save_frame2(frame2)
                else:
                    print(f"[Warning] Iteration {i}: No frames available")
                
                print(f"[Capture] Iteration {i}/{num_frames-1}")
                time.sleep(interval)
            
            print(f"[Capture] Captured {num_frames} frames")
            return True
        
        except Exception as e:
            print(f"[Error] Failed to capture frames: {e}")
            return False
    
    def reset_frame_counters(self):
        """重置帧计数器"""
        self.frame_counter1 = 0
        self.frame_counter2 = 0
        print("[Info] Frame counters reset")
    
    def close(self):
        """关闭相机，释放资源"""
        print(f"[Cleanup] Closing dual camera...")
        
        try:
            if self.stream1 is not None or self.stream2 is not None:
                if self.stream1 is not None:
                    self.stream1.stop()
                    self.stream1.terminate()
                if self.stream2 is not None:
                    self.stream2.stop()
                    self.stream2.terminate()
                print(f"[Cleanup] Image streams stopped")
        except Exception as e:
            print(f"[Error] Error stopping streams: {e}")
        
        try:
            if self.sensor1 is not None or self.sensor2 is not None:
                if self.sensor1 is not None:
                    self.sensor1.register = None
                    self.sensor1.stop()
                    self.sensor1.terminate()
                if self.sensor2 is not None:
                    self.sensor2.register = None
                    self.sensor2.stop()
                    self.sensor2.terminate()
                print(f"[Cleanup] Sensors stopped")
        except Exception as e:
            print(f"[Error] Error stopping sensors: {e}")
        
        self.is_initialized = False
        print(f"[Cleanup] Dual camera closed")
    
    def __enter__(self):
        """上下文管理器入口"""
        self.initialize()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器出口"""
        self.close()
        return False

if __name__ == "__main__":
    """
    测试双相机类的示例代码
    """
    # 方法1：使用上下文管理器（推荐）
    print("="*50)
    print("方法1：使用上下文管理器")
    print("="*50)
    
    try:
        with DualCamera(lc_imgs_path="camera/lc_imgs/", 
                       rc_imgs_path="camera/rc_imgs/",
                       clear_on_init=True) as camera:
            # 采集5帧图像，每帧间隔0.5秒
            camera.capture_frames(num_frames=5, interval=0.5)
    
    except Exception as e:
        print(f"[Error] {e}")
        import traceback
        traceback.print_exc()
    
    # 方法2：手动初始化和关闭
    print("\n" + "="*50)
    print("方法2：手动初始化和关闭")
    print("="*50)
    
    camera = DualCamera(lc_imgs_path="camera/lc_imgs/", 
                       rc_imgs_path="camera/rc_imgs/",
                       clear_on_init=False)
    
    try:
        if camera.initialize():
            # 采集10帧图像，每帧间隔0.2秒
            camera.capture_frames(num_frames=10, interval=0.2)
    
    except KeyboardInterrupt:
        print("[Info] 用户中断")
    
    except Exception as e:
        print(f"[Error] {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        camera.close()