import threading
import time
import rtde_receive
import rtde_control
import numpy as np
from pynput import keyboard  # 使用pynput库来监听键盘输入
# https://github.com/hust-dhn/URScanWithUR.git
# ====================
# 参数
# ====================
# 需要提前更改参数，包括自由拖动中的质量质心、使能关节和权重
sensor_mass = 0.820  # 传感器质量，单位：千克
sensor_cog = [0.007, -0.006, 0.07]  # 质心位置，单位：米
free_axes = [1, 1, 1, 1, 1, 1]  #使能关节，1表示使能，0表示禁用
feature = [1, 1, 1, 0.5, 0.5, 0.5]  # 权重，用于调整每个关节的拖动力
# 关节角的伺服控制参数
joint_velocity = 0.5
joint_acceleration = 0.5
joint_gain = 300 #伺服控制增益
# 笛卡尔空间位置的伺服控制参数
cartesian_velocity = 0.5
cartesian_acceleration = 0.5
cartesian_gain = 300 #伺服控制增益

# ====================
# 初始化设置（无需更改）
# ====================
robot_ip = "192.168.253.101"
rtde_c = rtde_control.RTDEControlInterface(robot_ip)
rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)

start_drag_flag = False
stop_drag_flag = False
quit_flag = False
auto_callback_flag = False

target_joint = None
target_position = None

hz = 125
dt = 1.0/hz
lookahead_time = 5*dt


# ====================
# 监听键盘、自由拖动、伺服控制
# ====================
# 用于监听键盘输入的函数, 按下 'd' 键开始拖动，按下 'c' 键停止拖动，按下 'q' 键退出程序, 按下 't' 键等待外部力和回调
def listen_keyboard():
    def on_press(key):
        global start_drag_flag, stop_drag_flag, quit_flag, auto_callback_flag
        try:
            if key.char == 'd':  # 按下 'd' 键开始拖动
                start_drag_flag = True
                stop_drag_flag = False
                auto_callback_flag = False
                print("=======开始拖动=======")
            elif key.char == 'c':  # 按下 'c' 键停止拖动
                start_drag_flag = False
                stop_drag_flag = True
                auto_callback_flag = False
                print("=======停止拖动=======")
            elif key.char == 'q':  # 按下 'q' 键退出程序
                start_drag_flag = False
                stop_drag_flag = False
                auto_callback_flag = False
                quit_flag = True
                print("=======退出程序=======")
                return False  # 退出监听器
            elif key.char == 't':  # 按下 't' 键
                start_drag_flag = False
                stop_drag_flag = False
                auto_callback_flag = True
                print("=======等待外部力和回调=======")
        except AttributeEqrror:
            pass  # 处理其他特殊按键
    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()  # 开始监听键盘事件

# 用于自由拖动机器人的函数
def drag_robot():
    global start_drag_flag, stop_drag_flag, quit_flag   
    while not quit_flag:
        if stop_drag_flag:
            # 结束Freedrive模式
            rtde_c.endFreedriveMode()
        if start_drag_flag:
            # 开始Freedrive模式
            rtde_c.freedriveMode(free_axes, feature)
        time.sleep(dt)  # 每0.1秒检查一次回调状态

# 用于自适应控制机器人的函数
def auto_callback():
    global auto_callback_flag, quit_flag, target_joint, target_position
    while not quit_flag:
        if auto_callback_flag:
            rtde_c.servoL(target_position, cartesian_velocity, cartesian_acceleration, hz, lookahead_time, cartesian_gain)
        time.sleep(dt)  # 每0.1秒检查一次回调状态

# ====================
# 主函数
# ====================
def main():
    global quit_flag
    # 设置 payload 和 CoG
    rtde_c.setPayload(sensor_mass, sensor_cog)
    print(f"=======payload: {sensor_mass}, CoG: {sensor_cog}=======")

    # 启动键盘监听线程
    keyboard_thread = threading.Thread(target=listen_keyboard)
    keyboard_thread.start()
    # 启动拖动机器人线程
    drag_thread = threading.Thread(target=drag_robot)
    drag_thread.start()
    # 启动自适应控制线程
    auto_thread = threading.Thread(target=auto_callback)
    auto_thread.start()

    while not quit_flag:
        time.sleep(dt)

    # 等待所有线程完成
    drag_thread.join()
    auto_thread.join()
    keyboard_thread.join()


if __name__ == "__main__":
    main()
