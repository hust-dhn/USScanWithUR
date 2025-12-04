import threading
import time
import rtde_receive
import rtde_control
import numpy as np
from pynput import keyboard  # 使用pynput库来监听键盘输入

# ====================
# 参数
# ====================
# 需要提前更改参数，包括自由拖动中的质量质心、使能关节和权重
sensor_mass = 0.820  # 传感器质量，单位：千克
sensor_cog = [0.007, -0.006, 0.07]  # 质心位置，单位：米
free_axes = [1, 1, 1, 1, 1, 1]  # 使能关节，1表示使能，0表示禁用
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

# 初始化目标位置为None
target_joint = None
target_position = None

hz = 125
dt = 1.0/hz
lookahead_time = 5*dt

# ====================
# 新增功能函数
# ====================
def set_target_to_current():
    """将当前位置设置为目标位置"""
    global target_position
    current_pose = rtde_r.getActualTCPPose()
    target_position = current_pose.copy()  # 使用copy避免引用问题
    print(f"目标位置已更新为当前位置: {target_position}")
    return target_position

def set_custom_target_position(new_position):
    """设置自定义目标位置"""
    global target_position
    if len(new_position) == 6:
        target_position = new_position.copy()
        print(f"目标位置已设置为: {target_position}")
        return True
    else:
        print("错误: 位置必须是6元素列表 [x, y, z, rx, ry, rz]")
        return False

def move_target_position(delta):
    """相对移动目标位置"""
    global target_position
    if target_position is not None:
        for i in range(6):
            target_position[i] += delta[i]
        print(f"目标位置已相对移动: {target_position}")
        return True
    else:
        print("错误: 请先设置目标位置")
        return False

def print_current_position():
    """打印当前位置"""
    current_pose = rtde_r.getActualTCPPose()
    print(f"当前位置: {current_pose}")
    return current_pose

# ====================
# 监听键盘、自由拖动、伺服控制
# ====================
def listen_keyboard():
    def on_press(key):
        global start_drag_flag, stop_drag_flag, quit_flag, auto_callback_flag, target_position
        
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
                
            elif key.char == 't':  # 按下 't' 键开始自适应控制
                if target_position is None:
                    # 如果没有目标位置，先设置为当前位置
                    set_target_to_current()
                start_drag_flag = False
                stop_drag_flag = False
                auto_callback_flag = True
                print("=======开始自适应控制=======")
                print(f"目标位置: {target_position}")
                
            elif key.char == 'p':  # 按下 'p' 键将当前位置设置为目标位置
                set_target_to_current()
                if auto_callback_flag:
                    print("目标位置已更新，自适应控制中...")
                    
            elif key.char == 'o':  # 按下 'o' 键打印当前位置
                print_current_position()
                
            elif key.char == 'u':  # 按下 'u' 键向上移动目标位置 5cm
                if target_position is not None:
                    move_target_position([0, 0, 0.05, 0, 0, 0])
                else:
                    print("请先设置目标位置（按't'或'p'）")
                    
            elif key.char == 'i':  # 按下 'i' 键向下移动目标位置 5cm
                if target_position is not None:
                    move_target_position([0, 0, -0.05, 0, 0, 0])
                else:
                    print("请先设置目标位置（按't'或'p'）")
                    
            elif key.char == 'j':  # 按下 'j' 键向左移动目标位置 5cm
                if target_position is not None:
                    move_target_position([-0.05, 0, 0, 0, 0, 0])
                else:
                    print("请先设置目标位置（按't'或'p'）")
                    
            elif key.char == 'l':  # 按下 'l' 键向右移动目标位置 5cm
                if target_position is not None:
                    move_target_position([0.05, 0, 0, 0, 0, 0])
                else:
                    print("请先设置目标位置（按't'或'p'）")
                    
            elif key.char == 'k':  # 按下 'k' 键向前移动目标位置 5cm
                if target_position is not None:
                    move_target_position([0, 0.05, 0, 0, 0, 0])
                else:
                    print("请先设置目标位置（按't'或'p'）")
                    
            elif key.char == 'm':  # 按下 'm' 键向后移动目标位置 5cm
                if target_position is not None:
                    move_target_position([0, -0.05, 0, 0, 0, 0])
                else:
                    print("请先设置目标位置（按't'或'p'）")
                    
        except AttributeError:
            pass  # 处理其他特殊按键
            
    print("\n=======键盘控制说明=======")
    print("d: 开始自由拖动")
    print("c: 停止拖动")
    print("t: 开始自适应控制")
    print("p: 将当前位置设置为目标位置")
    print("o: 打印当前位置")
    print("u: 目标位置上移5cm")
    print("i: 目标位置下移5cm")
    print("j: 目标位置左移5cm")
    print("l: 目标位置右移5cm")
    print("k: 目标位置前移5cm")
    print("m: 目标位置后移5cm")
    print("q: 退出程序")
    print("========================\n")
    
    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()  # 开始监听键盘事件

def drag_robot():
    global start_drag_flag, stop_drag_flag, quit_flag   
    while not quit_flag:
        if stop_drag_flag:
            # 结束Freedrive模式
            rtde_c.endFreedriveMode()
            stop_drag_flag = False
        if start_drag_flag:
            # 开始Freedrive模式
            rtde_c.freedriveMode(free_axes, feature)
        time.sleep(dt)  # 每0.1秒检查一次回调状态

def auto_callback():
    global auto_callback_flag, quit_flag, target_joint, target_position
    while not quit_flag:
        if auto_callback_flag and target_position is not None:
            try:
                rtde_c.servoL(target_position, cartesian_velocity, cartesian_acceleration, 
                            hz, lookahead_time, cartesian_gain)
            except Exception as e:
                print(f"伺服控制错误: {e}")
                auto_callback_flag = False
        time.sleep(dt)  # 每0.1秒检查一次回调状态

# ====================
# 主函数
# ====================
def main():
    global quit_flag, target_position
    
    # 设置 payload 和 CoG
    rtde_c.setPayload(sensor_mass, sensor_cog)
    print(f"=======payload: {sensor_mass}, CoG: {sensor_cog}=======")
    
    # 设置初始目标位置为当前位置
    set_target_to_current()

    # 启动键盘监听线程
    keyboard_thread = threading.Thread(target=listen_keyboard)
    keyboard_thread.daemon = True
    keyboard_thread.start()
    
    # 启动拖动机器人线程
    drag_thread = threading.Thread(target=drag_robot)
    drag_thread.daemon = True
    drag_thread.start()
    
    # 启动自适应控制线程
    auto_thread = threading.Thread(target=auto_callback)
    auto_thread.daemon = True
    auto_thread.start()

    try:
        while not quit_flag:
            time.sleep(dt)
    except KeyboardInterrupt:
        print("程序被用户中断")
        quit_flag = True

    # 清理资源
    print("正在停止机器人...")
    rtde_c.servoStop()
    rtde_c.stopScript()
    rtde_c.disconnect()
    rtde_r.disconnect()
    print("机器人已停止，程序退出")

if __name__ == "__main__":
    main()