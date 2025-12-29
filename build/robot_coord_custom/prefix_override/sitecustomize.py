import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/zzz/ros2_ws/src/my_ur10e_control/install/robot_coord_custom'
