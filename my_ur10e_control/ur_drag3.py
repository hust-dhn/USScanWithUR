import threading
import time
import rtde_receive
import rtde_control
import numpy as np
from scipy.spatial.transform import Rotation as R
from pynput import keyboard

# ====================
# 参数
# ====================
sensor_mass = 0.820
sensor_cog = [0.007, -0.006, 0.07]
free_axes = [1, 1, 1, 1, 1, 1]
feature = [1, 1, 1, 0.5, 0.5, 0.5]

# 工具参数 - 必须准确测量
# 格式: [dx, dy, dz, drx, dry, drz] (米, 弧度)
# 工具末端相对于机械臂末端法兰的偏移
tool_offset = [0.0, -0.07625, 0.07708, 0.0, 0.0, 0.0]  # 假设工具在Z方向偏移150mm

joint_velocity = 0.5
joint_acceleration = 0.5
joint_gain = 300

cartesian_velocity = 0.5
cartesian_acceleration = 0.5
cartesian_gain = 300

# ====================
# 初始化设置
# ====================
robot_ip = "192.168.253.101"
rtde_c = rtde_control.RTDEControlInterface(robot_ip)
rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)

start_drag_flag = False
stop_drag_flag = False
quit_flag = False
auto_callback_flag = False

target_tool_pose = None  # 存储工具末端目标位置

hz = 125
dt = 1.0/hz
lookahead_time = 5*dt

# ====================
# 工具坐标系转换函数 - 使用正确的关系
# ====================
def pose_to_matrix(pose):
    """将位姿[x, y, z, rx, ry, rz]转换为齐次变换矩阵"""
    position = pose[:3]
    euler_angles = pose[3:]
    
    # 创建旋转矩阵（假设是绕固定轴XYZ旋转）
    rotation = R.from_euler('xyz', euler_angles, degrees=False)
    rot_matrix = rotation.as_matrix()
    
    # 构建齐次变换矩阵
    T = np.eye(4)
    T[:3, :3] = rot_matrix
    T[:3, 3] = position
    
    return T

def matrix_to_pose(T):
    """将齐次变换矩阵转换为位姿[x, y, z, rx, ry, rz]（弧度）"""
    position = T[:3, 3]
    rot_matrix = T[:3, :3]
    
    # 转换为欧拉角（绕固定轴XYZ，弧度）
    rotation = R.from_matrix(rot_matrix)
    euler_angles = rotation.as_euler('xyz', degrees=False)
    
    return np.concatenate([position, euler_angles])

def get_current_end_effector_pose():
    """
    获取当前机械臂末端法兰位姿
    当示教器工具偏移为0时，getActualTCPPose()返回的就是机械臂末端法兰位姿
    """
    return rtde_r.getActualTCPPose()

def get_current_tool_pose():
    """
    获取当前工具末端位姿
    通过将机械臂末端位姿转换为工具末端位姿
    """
    current_end_pose = get_current_end_effector_pose()
    return end_to_tool_transform(current_end_pose, tool_offset)

def end_to_tool_transform(end_pose, tool_offset):
    """
    将机械臂末端位姿转换为工具末端位姿
    
    参数:
    end_pose: 机械臂末端位姿 [x, y, z, rx, ry, rz]
    tool_offset: 工具末端相对于机械臂末端的偏移 [dx, dy, dz, drx, dry, drz]
    
    返回:
    tool_pose: 工具末端位姿
    """
    # 机械臂末端位姿矩阵
    T_end = pose_to_matrix(end_pose)
    
    # 工具相对于机械臂末端的变换矩阵
    T_tool_rel = pose_to_matrix(tool_offset)
    
    # 计算工具末端位姿矩阵: T_tool = T_end * T_tool_rel
    T_tool = np.dot(T_end, T_tool_rel)
    
    # 转换回位姿表示
    tool_pose = matrix_to_pose(T_tool)
    
    return tool_pose

def tool_to_end_transform(tool_pose, tool_offset):
    """
    将工具末端位姿转换为机械臂末端位姿
    
    参数:
    tool_pose: 工具末端位姿 [x, y, z, rx, ry, rz]
    tool_offset: 工具末端相对于机械臂末端的偏移 [dx, dy, dz, drx, dry, drz]
    
    返回:
    end_pose: 机械臂末端位姿
    """
    # 工具末端位姿矩阵
    T_tool = pose_to_matrix(tool_pose)
    
    # 工具相对于机械臂末端的变换矩阵
    T_tool_rel = pose_to_matrix(tool_offset)
    
    # 计算机械臂末端位姿矩阵: T_end = T_tool * inv(T_tool_rel)
    T_end = np.dot(T_tool, np.linalg.inv(T_tool_rel))
    
    # 转换回位姿表示
    end_pose = matrix_to_pose(T_end)
    
    return end_pose

# ====================
# 功能函数
# ====================
def set_target_to_current():
    """将当前位置（工具末端）设置为目标位置"""
    global target_tool_pose
    current_tool_pose = get_current_tool_pose()
    target_tool_pose = current_tool_pose.copy()
    print(f"工具末端目标位置已更新为当前位置: {target_tool_pose}")
    return target_tool_pose

def set_custom_target_position(new_tool_pose):
    """设置自定义工具末端目标位置"""
    global target_tool_pose
    if len(new_tool_pose) == 6:
        target_tool_pose = new_tool_pose.copy()
        print(f"工具末端目标位置已设置为: {target_tool_pose}")
        return True
    else:
        print("错误: 位置必须是6元素列表 [x, y, z, rx, ry, rz]")
        return False

def move_target_position(delta):
    """相对移动工具末端目标位置"""
    global target_tool_pose
    if target_tool_pose is not None:
        for i in range(6):
            target_tool_pose[i] += delta[i]
        print(f"工具末端目标位置已相对移动: {target_tool_pose}")
        return True
    else:
        print("错误: 请先设置目标位置")
        return False

def print_current_position():
    """打印当前位置（工具末端）"""
    current_tool_pose = get_current_tool_pose()
    print(f"工具末端当前位置: {current_tool_pose}")
    return current_tool_pose

def print_end_effector_position():
    """打印机械臂末端当前位置"""
    current_end_pose = get_current_end_effector_pose()
    print(f"机械臂末端当前位置: {current_end_pose}")
    return current_end_pose

def input_target_position():
    """从控制台输入工具末端目标位置"""
    print("\n请输入工具末端目标位置 (单位: 米, 弧度)")
    print("格式: x y z rx ry rz")
    print("例如: 0.3 0.1 0.2 0 3.14 0")
    
    try:
        user_input = input("请输入工具末端目标位置: ")
        values = user_input.split()
        
        if len(values) != 6:
            print("错误: 请输入6个数值")
            return None
        
        position = [float(val) for val in values]
        return position
        
    except ValueError:
        print("错误: 输入的不是有效数字")
        return None

def test_tool_calibration():
    """
    测试工具校准是否正确
    这个函数可以帮助验证工具偏移是否正确
    """
    print("\n=======工具校准测试=======")
    
    # 获取当前机械臂末端位姿
    current_end = get_current_end_effector_pose()
    print(f"机械臂末端位姿: {current_end}")
    
    # 计算工具末端位姿
    current_tool = get_current_tool_pose()
    print(f"计算得到的工具末端位姿: {current_tool}")
    
    # 反向计算验证
    end_from_tool = tool_to_end_transform(current_tool, tool_offset)
    print(f"从工具位姿反算的机械臂末端位姿: {end_from_tool}")
    
    # 检查误差
    error = np.array(end_from_tool) - np.array(current_end)
    print(f"误差: {error}")
    print("测试完成")
    
    return error

def simple_tool_calibration():
    """
    简单的四点法工具校准
    这种方法可以找到工具偏移
    """
    print("\n=======简单工具校准=======")
    print("请将工具末端对准一个固定点")
    print("然后以不同的姿态记录四个位置")
    
    poses = []
    
    for i in range(4):
        input(f"将机器人移动到第{i+1}个姿态，然后按Enter键...")
        pose = get_current_end_effector_pose()
        poses.append(pose)
        print(f"记录第{i+1}个位姿: {pose}")
    
    print("\n记录的位姿:")
    for i, pose in enumerate(poses):
        print(f"位姿{i+1}: {pose}")
    
    # 这里应该实现四点法求解工具偏移
    # 由于四点法需要解方程，这里提供一个简化版本
    print("\n请根据测量结果手动计算工具偏移:")
    print("1. 工具在机械臂末端的坐标偏移")
    print("2. 工具相对于机械臂末端的姿态偏移")
    
    return poses

# ====================
# 监听键盘
# ====================
def listen_keyboard():
    def on_press(key):
        global start_drag_flag, stop_drag_flag, quit_flag, auto_callback_flag, target_tool_pose, tool_offset
        
        try:
            if key.char == 'd':  # 开始拖动
                start_drag_flag = True
                stop_drag_flag = False
                auto_callback_flag = False
                print("=======开始拖动=======")
                
            elif key.char == 'c':  # 停止拖动
                start_drag_flag = False
                stop_drag_flag = True
                auto_callback_flag = False
                print("=======停止拖动=======")
                
            elif key.char == 'q':  # 退出程序
                quit_flag = True
                print("=======退出程序=======")
                return False
                
            elif key.char == 't':  # 开始自适应控制
                start_drag_flag = False
                stop_drag_flag = True
                auto_callback_flag = False
                
                new_position = input_target_position()
                
                if new_position is not None:
                    if set_custom_target_position(new_position):
                        auto_callback_flag = True
                        print("=======开始自适应控制=======")
                        print(f"工具末端目标位置: {target_tool_pose}")
                    else:
                        print("设置目标位置失败")
                else:
                    print("未输入有效目标位置")
                
            elif key.char == 'p':  # 将当前位置设置为目标位置
                current_tool_pose = get_current_tool_pose()
                if set_custom_target_position(current_tool_pose):
                    print("工具末端目标位置已设置为当前位置")
                    if not auto_callback_flag:
                        print("提示: 按't'键开始自适应控制移动到该位置")
                
            elif key.char == 'o':  # 打印工具末端当前位置
                print_current_position()
                
            elif key.char == 'e':  # 打印机械臂末端当前位置
                print_end_effector_position()
                
            elif key.char == '1':  # 测试工具校准
                test_tool_calibration()
                
            elif key.char == '2':  # 简单工具校准
                simple_tool_calibration()
                
            elif key.char == 'u':  # 工具末端目标位置上移
                if target_tool_pose is not None:
                    move_target_position([0, 0, 0.05, 0, 0, 0])
                    if auto_callback_flag:
                        print("工具末端目标位置已更新")
                else:
                    print("请先设置目标位置")
                    
            elif key.char == 'i':  # 工具末端目标位置下移
                if target_tool_pose is not None:
                    move_target_position([0, 0, -0.05, 0, 0, 0])
                    if auto_callback_flag:
                        print("工具末端目标位置已更新")
                else:
                    print("请先设置目标位置")
                    
            elif key.char == 'j':  # 工具末端目标位置左移
                if target_tool_pose is not None:
                    move_target_position([-0.05, 0, 0, 0, 0, 0])
                    if auto_callback_flag:
                        print("工具末端目标位置已更新")
                else:
                    print("请先设置目标位置")
                    
            elif key.char == 'l':  # 工具末端目标位置右移
                if target_tool_pose is not None:
                    move_target_position([0.05, 0, 0, 0, 0, 0])
                    if auto_callback_flag:
                        print("工具末端目标位置已更新")
                else:
                    print("请先设置目标位置")
                    
            elif key.char == 'k':  # 工具末端目标位置前移
                if target_tool_pose is not None:
                    move_target_position([0, 0.05, 0, 0, 0, 0])
                    if auto_callback_flag:
                        print("工具末端目标位置已更新")
                else:
                    print("请先设置目标位置")
                    
            elif key.char == 'm':  # 工具末端目标位置后移
                if target_tool_pose is not None:
                    move_target_position([0, -0.05, 0, 0, 0, 0])
                    if auto_callback_flag:
                        print("工具末端目标位置已更新")
                else:
                    print("请先设置目标位置")
                    
        except AttributeError:
            pass
            
    print("\n=======键盘控制说明=======")
    print("d: 开始自由拖动")
    print("c: 停止拖动")
    print("t: 设置工具末端目标位置并开始自适应控制")
    print("p: 将工具末端当前位置设置为目标位置")
    print("o: 打印工具末端当前位置")
    print("e: 打印机械臂末端当前位置")
    print("1: 测试工具校准")
    print("2: 简单工具校准")
    print("u: 工具末端目标位置上移5cm")
    print("i: 工具末端目标位置下移5cm")
    print("j: 工具末端目标位置左移5cm")
    print("l: 工具末端目标位置右移5cm")
    print("k: 工具末端目标位置前移5cm")
    print("m: 工具末端目标位置后移5cm")
    print("q: 退出程序")
    print(f"当前工具偏移: {tool_offset}")
    print("=========================\n")
    
    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()

def drag_robot():
    global start_drag_flag, stop_drag_flag, quit_flag   
    while not quit_flag:
        if stop_drag_flag:
            rtde_c.endFreedriveMode()
            stop_drag_flag = False
        if start_drag_flag:
            rtde_c.freedriveMode(free_axes, feature)
        time.sleep(dt)

def auto_callback():
    """
    自适应控制线程
    将工具末端目标位置转换为机械臂末端目标位置，然后发送给机器人
    """
    global auto_callback_flag, quit_flag, target_tool_pose
    
    while not quit_flag:
        if auto_callback_flag and target_tool_pose is not None:
            try:
                # 将工具末端目标位置转换为机械臂末端目标位置
                end_target_pose = tool_to_end_transform(target_tool_pose, tool_offset)
                
                # 使用servoL进行平滑控制
                rtde_c.servoL(end_target_pose, cartesian_velocity, cartesian_acceleration, 
                             hz, lookahead_time, cartesian_gain)
                
            except Exception as e:
                print(f"伺服控制错误: {e}")
                auto_callback_flag = False
        time.sleep(dt)

# ====================
# 主函数
# ====================
def main():
    global quit_flag, target_tool_pose
    
    # 设置 payload 和 CoG
    rtde_c.setPayload(sensor_mass, sensor_cog)
    print(f"=======payload: {sensor_mass}, CoG: {sensor_cog}=======")
    
    # 重要说明
    print("\n=== 重要说明 ===")
    print("当前模式: 示教器工具偏移为0")
    print("这意味着:")
    print("1. UR返回的是机械臂末端法兰位姿")
    print("2. 代码中需要将工具末端位姿转换为机械臂末端位姿")
    print("3. 请确保工具偏移参数准确: {tool_offset}")
    print("================\n")
    
    # 测试当前位姿
    print("测试当前位姿:")
    end_pose = get_current_end_effector_pose()
    tool_pose = get_current_tool_pose()
    print(f"机械臂末端位姿: {end_pose}")
    print(f"工具末端位姿: {tool_pose}")
    
    # 设置初始目标位置为当前位置（工具末端）
    set_target_to_current()

    # 启动线程
    keyboard_thread = threading.Thread(target=listen_keyboard)
    keyboard_thread.daemon = True
    keyboard_thread.start()
    
    drag_thread = threading.Thread(target=drag_robot)
    drag_thread.daemon = True
    drag_thread.start()
    
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