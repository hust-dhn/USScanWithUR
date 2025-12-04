import threading
import time
import rtde_receive
import rtde_control
import numpy as np
from pynput import keyboard  # 使用pynput库来监听键盘输入

# 机器人IP
robot_ip = "192.168.253.102"
# RTDE接口初始化
rtde_c = rtde_control.RTDEControlInterface(robot_ip)
rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)

# 全局变量，用于存储上一次的力矩值、最初的力矩值和差值的平方和
last_force = None
initial_force = None  # 保存最初的力矩
force_diff_squared_sum = 0
writing_force = False  # 标志是否正在写力
quit_flag = False  # 标志是否退出程序

# 用于存储力和力矩的差值的函数
def write_force_difference_to_file():
    global last_force, force_diff_squared_sum, initial_force, writing_force
    while True:
        # 获取当前TCP的力
        current_force = np.array(rtde_r.getActualTCPForce())

        if writing_force:
            print("--------------------正在写力---------------------")
            # 如果没有初始化最初力矩，则设定当前力矩为初始力矩
            if initial_force is None:
                initial_force = current_force

            # 计算当前力矩与上次力矩的差值
            if last_force is not None:
                force_diff = current_force - last_force
                force_diff_squared_sum = np.sum(np.square(force_diff))  # 计算差值的平方和

            # 将当前力矩写入文件
            with open("diff.txt", "a") as file:
                force_str = [f"{val:.2f}" for val in force_diff]  # 保留两位小数
                file.write(f"Force: {', '.join(force_str)}\n")  # 将当前力矩写入文件

            with open("current.txt", "a") as file:
                force_str = [f"{val:.2f}" for val in current_force]  # 保留两位小数
                file.write(f"Force: {', '.join(force_str)}\n")  # 将当前力矩写入文件                

            # 计算力和最初力矩的差值，并保存到current.txt
            force_difference_from_initial = current_force - initial_force
            with open("caliborate.txt", "a") as file:
                force_diff_initial_str = [f"{val:.2f}" for val in force_difference_from_initial]  # 保留两位小数
                file.write(f"Force Difference from Initial: {', '.join(force_diff_initial_str)}\n")

        last_force = current_force  # 更新上一次的力矩值
        time.sleep(0.1)  # 以0.1秒的间隔获取数据，频率较快

# 用于判断力矩变化的函数，并写入状态文件
def write_state_based_on_force():
    global force_diff_squared_sum
    while True:
        # 获取当前时间
        current_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        
        # 判断力矩差值的平方和是否小于5
        if force_diff_squared_sum < 5:
            state_message = "力没变化"
        else:
            state_message = "力有变化"
        
        # 写入状态到state.txt文件
        with open("state.txt", "a") as file:
            file.write(f"{current_time} {state_message}\n")
        
        time.sleep(1)  # 每秒判断一次

# 用于监听键盘输入的函数
def listen_keyboard():
    def on_press(key):
        global writing_force, quit_flag
        try:
            if key.char == 'd':  # 按下 'd' 键开始写力
                writing_force = True
                print("开始写力")
            elif key.char == 'c':  # 按下 'c' 键停止写力
                writing_force = False
                print("停止写力")
            elif key.char == 'q':  # 按下 'q' 键退出程序
                quit_flag = True
                print("退出程序")
                return False  # 退出监听器
        except AttributeError:
            pass  # 忽略非字符按键的事件

    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()  # 开始监听键盘事件

# 用于打印当前时间的函数
def print_time():
    while True:
        current_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        print(f"当前时间: {current_time}")
        time.sleep(1)  # 每秒打印一次

# 启动力矩差值写入文件的线程
force_diff_thread = threading.Thread(target=write_force_difference_to_file, daemon=True)
force_diff_thread.start()

# 启动状态判断和写入的线程
state_thread = threading.Thread(target=write_state_based_on_force, daemon=True)
state_thread.start()

# 启动键盘监听的线程
keyboard_thread = threading.Thread(target=listen_keyboard, daemon=True)
keyboard_thread.start()

# 启动打印时间的线程
time_thread = threading.Thread(target=print_time, daemon=True)
time_thread.start()

# 保持主线程运行，否则程序会立即退出
while not quit_flag:
    time.sleep(1)  # 主线程保持活跃状态
