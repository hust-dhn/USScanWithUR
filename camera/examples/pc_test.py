from sys_path import *
from api.library_loader import *
from api.errors import ErrorCode, Error
from api.inu_sensor import InuSensor, ConnectionState, create_started_sensor
from api.point_cloud import PointCloudStream, PointCloudFrame, PointCloudOutputFormat
from api.shared import StreamType
import numpy as np
import time
from pynput import keyboard
import threading
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

stop_flag = False
current_points = None

def on_press(key):
    global stop_flag
    if key == keyboard.KeyCode.from_char('q'):
        stop_flag = True
        print("Stopping capture...")

def visualize_thread():
    global current_points, stop_flag
    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Real-time Point Cloud')
    
    while not stop_flag:
        if current_points is not None and len(current_points) > 0:
            ax.clear()
            ax.scatter(current_points[:, 0], current_points[:, 1], current_points[:, 2], s=1, c='b', alpha=0.5)
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            ax.set_title('Real-time Point Cloud')
            plt.draw()
            plt.pause(0.1)
        time.sleep(0.1)
    
    plt.close(fig)

if __name__ == "__main__":
    def _sensor_callback_func(self, sensor: InuSensor, connection_state: ConnectionState, error: Error) -> None:
        print(f'Sensor callback ConnectionState={connection_state} Error={error}')

    print("Starting point cloud capture... Press 'q' to stop.")

    # 初始化传感器
    sensor, channels_size, hw_information = create_started_sensor(_sensor_callback_func)
    print("Sensor initialized")

    # 创建点云流
    stream = sensor.create_started_stream(StreamType.POINT_CLOUD, None, None)
    print("Point cloud stream initialized")

    # 等待一秒以确保流稳定
    time.sleep(1)

    # 启动可视化线程
    vis_thread = threading.Thread(target=visualize_thread)
    vis_thread.start()

    # 启动键盘监听
    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    frame_count = 0
    while not stop_flag:
        # 获取一帧点云
        frame = stream.frame
        if frame is None or frame.num_of_points == 0:
            print("No valid point cloud frame captured")
            time.sleep(0.1)
            continue
        else:
            print(f"Captured point cloud {frame_count} with {frame.num_of_points} points")

            # 更新当前点云用于可视化
            points = frame.data
            current_points = points.reshape(frame.num_of_points, 3)

            # 保存点云数据到TXT文件
            output_file = f"point_cloud_{frame_count}.txt"
            with open(output_file, 'w') as f:
                f.write("# Point Cloud Data\n")
                f.write("# Format: X Y Z\n")
                for i in range(frame.num_of_points):
                    x = points[i * 3 + 0]
                    y = points[i * 3 + 1]
                    z = points[i * 3 + 2]
                    f.write(f"{x} {y} {z}\n")
            print(f"Point cloud saved to {output_file}")
            frame_count += 1

        time.sleep(0.1)  # 短暂延迟避免占用过多CPU

    # 停止监听
    listener.stop()
    listener.join()

    # 等待可视化线程结束
    vis_thread.join()

    # 终止流和传感器
    stream.terminate()
    sensor.terminate()
    print("Done")
