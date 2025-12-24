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

# ===== 单相机配置 =====
DEFAULT_IMAGE_FOLDER = 'camera/imgs/'          # 单相机图像文件夹
DEFAULT_CLEAR_ON_INIT = True  # 初始化时是否清空已有图像文件

class SingleCamera:
    """单相机封装类，用于超声扫描系统的图像采集"""
    
    def __init__(self, imgs_path=DEFAULT_IMAGE_FOLDER, clear_on_init=DEFAULT_CLEAR_ON_INIT, sensor_id=''):
        """
        初始化单相机类
        
        :param imgs_path: 图像保存路径
        :param clear_on_init: 初始化时是否清空已有图像文件
        :param sensor_id: 传感器ID，默认为空表示第一个相机
        """
        self.imgs_path = imgs_path
        self.clear_on_init = clear_on_init
        self.sensor_id = sensor_id
        
        # 相机对象
        self.sensor = None
        self.stream = None
        
        # 帧计数器
        self.frame_counter = 0
        self.channels_size = None
        
        # 是否初始化
        self.is_initialized = False
    
    def _sensor_callback_func(self, sensor: InuSensor, connection_state: ConnectionState, error: Error) -> None:
        """传感器回调函数"""
        print(f'[Camera] Sensor callback ConnectionState={connection_state} Error={error}')
    
    def _save_frame(self, frame: ImageFrame) -> None:
        """保存相机帧"""
        if frame is None:
            print('[Camera] Invalid frame')
        else:
            print(f"\t[Camera] Frame {frame.frame_index} - Format: {frame.format}, Size: {frame.width}x{frame.height}")
            
            if frame.buffer is not None:
                # 将缓冲区转换为图像
                depth_image = np.array(frame.buffer, dtype=np.uint8).reshape(frame.height, frame.width, 4)
                
                # 保存图像
                image_filename = f"{self.imgs_path}{self.frame_counter}.jpg"
                cv2.imwrite(image_filename, depth_image)
                print(f"[Camera] Saved frame {self.frame_counter} as {image_filename}")
            
            self.frame_counter += 1
    
    def _stream_callback_func(self, stream: ImageStream, frame: ImageFrame, error: Error) -> None:
        """数据流回调函数"""
        if error is None:
            print(f'[Camera Stream] Undefined error')
        elif error.code != ErrorCode.STATUS_OK:
            print(f'[Camera Stream] Error = {error.code} {error.description}')
        else:
            self._save_frame(frame)
    
    def initialize(self):
        """初始化相机"""
        try:
            # 清空图像文件
            if self.clear_on_init:
                delete_jpg_files(self.imgs_path)
                print(f"[Init] Cleared image files from {self.imgs_path}")
            
            print(f"[Init] Initializing camera...")
            
            # 初始化传感器
            self.sensor = InuSensor(self.sensor_id, '')
            
            # 初始化传感器
            hw_information, dpe_params = self.sensor.init()
            print(f"[Init] Sensor initialized: {type(hw_information)}")
            
            # 启动传感器
            self.channels_size = self.sensor.start(None, dpe_params)
                    
            # 注册传感器回调
            self.sensor.register = self._sensor_callback_func
            print(f"[Init] Sensor callback registered")
            
            # 创建并启动图像流
            self.stream = self.sensor.create_stream(StreamType.GENERAL_CAMERA)
            
            self.stream.init()
            self.stream.start()
            
            print(f"[Init] Image stream started")
            
            self.is_initialized = True
            print(f"[Init] Camera initialization completed successfully")
            return True
        
        except Exception as e:
            print(f"[Error] Failed to initialize camera: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def get_frame(self):
        """
        获取当前帧（非阻塞）
        
        :return: 当前帧，如果无可用帧则返回None
        """
        if not self.is_initialized:
            print("[Error] Camera not initialized")
            return None
        
        try:
            frame = self.stream.frame
            return frame
        except Exception as e:
            print(f"[Error] Failed to get frame: {e}")
            return None
    
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
        实时显示相机画面
        
        :param duration: 显示时长（秒），None 表示持续显示直到按 'q' 键退出
        :param window_width: 窗口宽度（像素）
        :param window_height: 窗口高度（像素）
        :return: 是否成功
        """
        if not self.is_initialized:
            print("[Error] Camera not initialized")
            return False
        
        # 创建窗口
        window_name = "Camera Live View"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, window_width, window_height)
        
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
                frame = self.get_frame()
                
                if frame is not None:
                    # 转换为 OpenCV 可显示的格式
                    img = self._frame_to_image(frame)
                    
                    if img is not None:
                        cv2.imshow(window_name, img)
                else:
                    print(f"[Warning] No frame available at {time.time() - start_time:.2f}s")
                
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
                frame = self.get_frame()
                
                if frame:
                    self._save_frame(frame)
                else:
                    print(f"[Warning] Iteration {i}: No frame available")
                
                print(f"[Capture] Iteration {i}/{num_frames-1}")
                time.sleep(interval)
            
            print(f"[Capture] Captured {num_frames} frames")
            return True
        
        except Exception as e:
            print(f"[Error] Failed to capture frames: {e}")
            return False
    
    def reset_frame_counter(self):
        """重置帧计数器"""
        self.frame_counter = 0
        print("[Info] Frame counter reset")
    
    def close(self):
        """关闭相机，释放资源"""
        print(f"[Cleanup] Closing camera...")
        
        try:
            if self.stream is not None:
                self.stream.stop()
                self.stream.terminate()
                print(f"[Cleanup] Image stream stopped")
        except Exception as e:
            print(f"[Error] Error stopping stream: {e}")
        
        try:
            if self.sensor is not None:
                self.sensor.register = None
                self.sensor.stop()
                self.sensor.terminate()
                print(f"[Cleanup] Sensor stopped")
        except Exception as e:
            print(f"[Error] Error stopping sensor: {e}")
        
        self.is_initialized = False
        print(f"[Cleanup] Camera closed")
    
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
    测试单相机类的示例代码
    """
    # 方法1：使用上下文管理器（推荐）
    print("="*50)
    print("方法1：使用上下文管理器")
    print("="*50)
    
    try:
        with SingleCamera(imgs_path=DEFAULT_IMAGE_FOLDER,
                         clear_on_init=DEFAULT_CLEAR_ON_INIT) as camera:
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
    
    camera = SingleCamera(imgs_path=DEFAULT_IMAGE_FOLDER,
                         clear_on_init=DEFAULT_CLEAR_ON_INIT)
    
    try:
        if camera.initialize():
            # 显示实时画面3秒
            camera.display_live_view(duration=3)
            
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