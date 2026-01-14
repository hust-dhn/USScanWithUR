import numpy as np
import math
import sys
import os

# 导入你提供的类
# 确保 ur10e_kine.py 在同一目录下
try:
    from kinematics.ur10e_kine import UR10eKine
except ImportError:
    print("错误: 找不到 ur10e_kine.py，请确保文件在同一目录下。")
    sys.exit(1)

def batch_process_ik(input_file, output_file):
    """
    读取位姿文件，计算逆运动学，输出关节角文件
    """
    
    # 1. 检查输入文件是否存在
    if not os.path.exists(input_file):
        print(f"错误: 输入文件 {input_file} 不存在。")
        return

    print("正在初始化 UR10e 运动学类...")
    print("注意: 该类初始化时会尝试连接机器人IP及读取 T_end2tcp.yaml，请确保环境配置正确。")
    
    try:
        # 初始化实例
        # 注意：根据你的代码，这里会尝试连接 '192.168.253.101' 并读取 yaml
        ur_solver = UR10eKine()
    except Exception as e:
        print(f"初始化失败: {e}")
        print("提示: 如果你是离线测试，可能需要注释掉 ur10e_kine.py 中关于 rtde_control 的连接代码。")
        return

    # 2. 读取输入数据
    # 假设 txt 格式为每行: x, y, z, rx, ry, rz
    # 支持逗号分隔或空格分隔
    print(f"正在读取 {input_file} ...")
    try:
        # 尝试逗号分隔
        poses = np.loadtxt(input_file, delimiter=',')
    except:
        try:
            # 尝试空格分隔
            poses = np.loadtxt(input_file)
        except Exception as e:
            print(f"读取文件失败，请检查格式: {e}")
            return

    # 确保数据是二维的 (防止只有一行数据时报错)
    if poses.ndim == 1:
        poses = poses.reshape(1, -1)

    num_poses = poses.shape[0]
    print(f"共加载 {num_poses} 个位姿点。")

    joint_solutions = []

    # 3. 设置初始关节角猜测值
    # IK函数需要 curr_theta 来计算最近解。
    # 对于第一个点，我们给定一个常规的初始姿态 (弧度制)
    # 例如: [0, -90, 90, -90, -90, 0] 的弧度版
    last_theta = [0, -math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0]

    # 4. 循环处理
    for i, pose in enumerate(poses):
        if len(pose) != 6:
            print(f"警告: 第 {i+1} 行数据维度不对，跳过。")
            continue
        
        # 将 [x, y, z, rx, ry, rz] 转为 变换矩阵 T
        try:
            T_target = ur_solver.XYZRXRYRZ_to_Tmatrix(pose)
            
            # 计算逆运动学 (IK)
            # 关键点：传入 last_theta 作为参考，确保解的连续性
            current_solution = ur_solver.IK(T_target, last_theta)
            
            joint_solutions.append(current_solution)
            
            # 更新 last_theta，下一次计算基于当前结果寻找最近解
            last_theta = current_solution
            
        except Exception as e:
            print(f"计算第 {i+1} 行时出错: {e}")
            # 如果出错，可以填入一行0或者复制上一行，这里选择填入0以保持行数一致
            joint_solutions.append([0]*6)

        # 进度条
        if (i + 1) % 100 == 0:
            print(f"已处理 {i + 1}/{num_poses} ...")

    # 5. 保存结果
    # 保存格式：每行6个关节角，逗号分隔，保留6位小数
    print(f"正在保存结果到 {output_file} ...")
    np.savetxt(output_file, joint_solutions, fmt='%.6f', delimiter=',')
    print("处理完成！")

if __name__ == "__main__":
    # 输入文件名
    input_txt = "ur10e/cfg/poses.txt"   # 请修改为你的输入文件名
    # 输出文件名
    output_txt = "ur10e/cfg/joints.txt" # 结果将保存在这里
    
    batch_process_ik(input_txt, output_txt)