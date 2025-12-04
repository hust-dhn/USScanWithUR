from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
import time

# 机器人IP
robot_ip = "192.168.253.102"
# RTDE接口初始化
rtde_c = RTDEControl(robot_ip)
rtde_r = RTDEReceive(robot_ip)

# 设置自由移动轴，这里让机器人在 X 和 Y 方向自由移动，Z 方向固定，旋转轴也固定
free_axes = [1, 1, 1, 0, 0, 0]

# 设置参考坐标系，使用基座坐标系（可以使用 "base" 或者提供一个具体的位姿）
feature = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0]  # 偏移 100mm 在 X 方向

# 进入自由拖动模式
if rtde_c.freedriveMode():
    print("Robot is in freedrive mode.")

# 在自由拖动模式下，机器人可以手动移动，等你完成操作后退出自由拖动模式
time.sleep(100)  # 假设你手动操作了 10 秒

# 退出自由拖动模式，恢复正常的控制模式
if rtde_c.endFreedriveMode():
    print("Robot is out of freedrive mode.")