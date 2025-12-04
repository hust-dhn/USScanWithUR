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

initial_force = None
last_force = None
start_drag = False
stop_drag = False
quit_flag = False
auto_callback_flag = False

# 用于监听键盘输入的函数
# 用于监听键盘输入的函数
def listen_keyboard():
    def on_press(key):
        global start_drag, stop_drag, quit_flag, auto_callback_flag
        try:
            if key.char == 'd':  # 按下 'd' 键开始拖动
                start_drag = True
                stop_drag = False
                print("开始拖动")

            elif key.char == 'c':  # 按下 'c' 键停止拖动
                start_drag = False
                stop_drag = True
                print("停止拖动")

            elif key.char == 'q':  # 按下 'q' 键退出程序
                quit_flag = True
                print("退出程序")
                return False  # 退出监听器

            elif key.char == 't':  # 按下 't' 键
                start_drag = False
                stop_drag = False
                auto_callback_flag = True
                print("等待外部力和回调")

        except AttributeError:
            pass  # 处理其他特殊按键

    # 使用pynput的Listener监听键盘事件
    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()  # 开始监听键盘事件

import numpy as np
import time

def drag_robot():
    global start_drag, stop_drag, quit_flag, last_force, initial_force
    M = np.array([2.0, 2.0, 2.0, 0.2, 0.2, 0.2])
    D = np.array([1.0, 1.0, 1.0, 0.5, 0.5, 0.5])
    K = np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5])

    start_time = time.time()    
    vd_t0 = np.array([0, 0, 0, 0, 0, 0])
    vc_t0 = np.array([0, 0, 0, 0, 0, 0])
    xd_t0 = np.array([0, 0, 0, 0, 0, 0])
    T = 0.05  # 时间步长
    initial_force = np.array(rtde_r.getActualTCPForce())  # 确保是 numpy 数组
    last_force = None  # 在开始时需要初始化 last_force
    
    while not quit_flag:
        if stop_drag:
            start_time = time.time()    
            vd_t0 = np.array([0, 0, 0, 0, 0, 0])
            vc_t0 = np.array([0, 0, 0, 0, 0, 0])
            xd_t0 = np.array(rtde_r.getTargetTCPPose())
            T = 0.05  # 时间步长
            initial_force = np.array(rtde_r.getActualTCPForce())  # 确保是 numpy 数组

        if start_drag:
            # 拖动机器人的代码
            start_time = time.time()
            current_force = np.array(rtde_r.getActualTCPForce())  # 确保是 numpy 数组
            
            if last_force is None:
                last_force = initial_force  # 初始化 last_force

            force_diff = current_force - initial_force  # 确保是 numpy 数组，可以进行减法运算
            # print(f"力差: {force_diff}")
            
            if np.linalg.norm(force_diff) < 10.0:
                # 拖动机器人
                vc_t1 = np.array(rtde_r.getTargetTCPSpeed())  # 确保是 numpy 数组
                xc_t1 = np.array(rtde_r.getTargetTCPPose())  # 确保是 numpy 数组
                ac_t1 = (vc_t1 - vc_t0) / T

                temp_a_k = M + D*T + K*T**2
                temp_f_k = force_diff + M * ac_t1 + D *(vc_t1-vd_t0) + K * (xc_t1 - xd_t0 - vd_t0 * T)

                ad_t1 = temp_f_k / temp_a_k
                vd_t1 = vd_t0 + ad_t1 * T   # 计算速度
                xd_t1 = xd_t0 + vd_t1 * T   # 计算位置
                
                xd_t0 = xd_t1
                vd_t0 = vd_t1
                vc_t0 = vc_t1

                velocity = 0.5
                acceleration = 0.5
                dt = 1.0/125
                lookahead_time=5*dt
                gain = 300 #伺服控制增益

                target_pose = xd_t1.tolist()                
                rtde_c.servoL(target_pose, velocity, acceleration, dt ,lookahead_time, gain)
                print(f"加速度: {ad_t1}")
                # print(f"速度: {vd_t1}")
                T = time.time() - start_time

            last_force = current_force  # 更新 last_force

        time.sleep(0.001)  # 每0.001秒检查一次拖动状态



def auto_callback():
    global auto_callback_flag, quit_flag
    while not quit_flag:
        if auto_callback_flag:
            print("等待外部力和回调")
            time.sleep(1)
        time.sleep(0.1)  # 每0.1秒检查一次回调状态

def main():
    global quit_flag
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
        time.sleep(1)

    # 等待所有线程完成
    keyboard_thread.join()
    drag_thread.join()
    auto_thread.join()

if __name__ == "__main__":
    main()
