"""
UR10e机器人运动学
这是一个用于UR10e机器人运动学计算的Python类，实现正、逆运动学功能
"""

import math          # 数学运算库，用于三角函数等数学操作
import numpy as np   # 数值计算库，用于矩阵运算和数组操作
<<<<<<< HEAD

=======
import rospy         # ROS通信库，用于机器人操作系统通信
from std_msgs.msg import Float32MultiArray  # ROS消息类型，用于传输浮点数数组
>>>>>>> 9262b4f14817ac8e9d445ca81c2fd4d183d7b2d8

# 定义UR10eKine类，实现UR10e机器人的运动学功能
class UR10eKine:
    def __init__(self, left_ur_mode=True):
        # 初始化UR10e机器人参数
        # DH参数：a连杆长度，d连杆偏距，alpha连杆扭转角
        self.a = [0, -612.7, -571.55, 0, 0, 0]           # 连杆长度参数（mm）
        self.d = [180.7, 0, 0, 174.15, 119.85, 116.55]  # 连杆偏距参数（mm）
        self.alpha = [math.pi/2, 0, 0, math.pi/2, -math.pi/2, 0]  # 连杆扭转角参数（弧度）

        # 左右臂UR10e模式配置
        self.left_ur_mode = left_ur_mode    # 布尔值，表示是否使用左臂模式，默认True
        self.right_ur_mode = not self.left_ur_mode  # 布尔值，表示是否使用右臂模式，与左臂相反
<<<<<<< HEAD
        

    
=======
        self.joint_pub = None               # ROS发布器对象，初始化为空
        self.ros_pub()                      # 调用方法初始化ROS发布器

        # 运动学计算相关参数
        self.ik_iteration = 20              # 逆运动学求解的迭代次数
        self.motion_steps = 500             # 直线运动插补时的分段步数
        self.speed_factor = 0.5             # 运动速度系数
        
    def ros_pub(self):
        # 初始化ROS节点和发布器
        if self.left_ur_mode:
            # 如果是左臂模式，创建名为'left_ur10e_kine'的ROS节点
            rospy.init_node('left_ur10e_kine', anonymous=True)
            # 创建发布器，发布到'left_joints_angle'话题，消息类型为Float32MultiArray
            self.joint_pub = rospy.Publisher('left_joints_angle', Float32MultiArray, queue_size=10)
        else:
            # 如果是右臂模式，创建名为'right_ur10e_kine'的ROS节点
            rospy.init_node('right_ur10e_kine', anonymous=True)
            # 创建发布器，发布到'right_joints_angle'话题
            self.joint_pub = rospy.Publisher('right_joints_angle', Float32MultiArray, queue_size=10)
>>>>>>> 9262b4f14817ac8e9d445ca81c2fd4d183d7b2d8

    def THT(self, Theta, A, D, Alpha):
        # 计算DH参数下的齐次变换矩阵
        # Theta: 关节角度，A: 连杆长度，D: 连杆偏距，Alpha: 连杆扭转角
<<<<<<< HEAD
        T = np.asmatrix((
=======
        T = np.mat((
>>>>>>> 9262b4f14817ac8e9d445ca81c2fd4d183d7b2d8
            # 第一行：[cos(θ), -sin(θ)*cos(α), sin(α)*sin(θ), a*cos(θ)]
            [math.cos(Theta), -math.sin(Theta)*math.cos(Alpha), math.sin(Alpha)*math.sin(Theta), A*math.cos(Theta)],
            # 第二行：[sin(θ), cos(θ)*cos(α), -cos(θ)*sin(α), a*sin(θ)]
            [math.sin(Theta), math.cos(Theta)*math.cos(Alpha), -math.cos(Theta)*math.sin(Alpha), A*math.sin(Theta)],
            # 第三行：[0, sin(α), cos(α), d]
            [0, math.sin(Alpha), math.cos(Alpha), D],
            # 第四行：[0, 0, 0, 1]
            [0, 0, 0, 1]
        ))
        return T  # 返回4x4齐次变换矩阵

    def FK(self, theta: list):
        # UR10e正运动学计算
        # 计算相邻关节间的齐次变换矩阵T01到T56
        T01 = self.THT(theta[0], self.a[0], self.d[0], self.alpha[0])  # 基座到第一关节
        T12 = self.THT(theta[1], self.a[1], self.d[1], self.alpha[1])  # 第一关节到第二关节
        T23 = self.THT(theta[2], self.a[2], self.d[2], self.alpha[2])  # 第二关节到第三关节
        T34 = self.THT(theta[3], self.a[3], self.d[3], self.alpha[3])  # 第三关节到第四关节
        T45 = self.THT(theta[4], self.a[4], self.d[4], self.alpha[4])  # 第四关节到第五关节
        T56 = self.THT(theta[5], self.a[5], self.d[5], self.alpha[5])  # 第五关节到第六关节
            
        # 累积所有变换矩阵得到总变换矩阵
        T = np.matrix(T01 * T12 * T23 * T34 * T45 * T56 )
        return T  # 返回从基座到末端的总齐次变换矩阵

    def IK(self, T: np.matrix, curr_theta: list):
        # UR10e逆运动学计算
        # 提取目标位姿的旋转矩阵和平移向量
        nx = T[0, 0]  # 旋转矩阵第1行第1列元素
        ny = T[1, 0]  # 旋转矩阵第2行第1列元素
        nz = T[2, 0]  # 旋转矩阵第3行第1列元素
        ox = T[0, 1]  # 旋转矩阵第1行第2列元素
        oy = T[1, 1]  # 旋转矩阵第2行第2列元素
        oz = T[2, 1]  # 旋转矩阵第3行第2列元素
        ax = T[0, 2]  # 旋转矩阵第1行第3列元素
        ay = T[1, 2]  # 旋转矩阵第2行第3列元素
        az = T[2, 2]  # 旋转矩阵第3行第3列元素
        px = T[0, 3]  # 位置向量x分量
        py = T[1, 3]  # 位置向量y分量
        pz = T[2, 3]  # 位置向量z分量

        # 计算关节角1
        m = self.d[5] * ay - py  # 中间变量m
        n = ax * self.d[5] - px  # 中间变量n
        # 计算两个可能的theta1解
        theta11 = math.atan2(m, n) - math.atan2(self.d[3], np.sqrt((m ** 2 + n ** 2 - (self.d[3]) ** 2)))
        theta12 = math.atan2(m, n) - math.atan2(self.d[3], -np.sqrt((m ** 2 + n ** 2 - (self.d[3]) ** 2)))
        # 存储所有可能的theta1解，并调整到[-π, π]范围内
        t1=[theta11,theta11,theta11,theta11,theta12,theta12,theta12,theta12]
        for i in range (8):
            if t1[i] > math.pi:
                t1[i] = t1[i] - 2 * math.pi
            elif t1[i] < -math.pi:
                t1[i] = t1[i] + 2 * math.pi

        # 计算关节角5
        # 计算四个可能的theta5解
        theta51 = math.acos(ax * math.sin(theta11) - ay * math.cos(theta11))
        theta52 = -math.acos(ax * math.sin(theta11) - ay * math.cos(theta11))
        theta53 = math.acos(ax * math.sin(theta12) - ay * math.cos(theta12))
        theta54 = -math.acos(ax * math.sin(theta12) - ay * math.cos(theta12))
        # 存储所有可能的theta5解，并调整到[-π, π]范围内
        t5= [theta51,theta51,theta52,theta52,theta53,theta53,theta54,theta54]
        for i in range (8):
            if t5[i] > math.pi:
                t5[i] = t5[i] - 2 * math.pi
            elif t5[i] < -math.pi:
                t5[i] = t5[i] + 2 * math.pi

        # 计算关节角6
        t6= [0,0,0,0,0,0,0,0]
        for i in range (8):
            mm = nx * math.sin(t1[i]) - ny * math.cos(t1[i])  # 中间变量mm
            nn = ox * math.sin(t1[i]) - oy * math.cos(t1[i])  # 中间变量nn
            # 计算theta6
            t6[i] = math.atan2(mm, nn) - math.atan2(math.sin(t5[i]), 0)
            # 调整到[-π, π]范围内
            if t6[i] > math.pi:
                t6[i] = t6[i] - 2 * math.pi
            elif t6[i] < -math.pi:
                t6[i] = t6[i] + 2 * math.pi

        # 计算关节角3
        m=[0,0,0,0,0,0,0,0]  # 中间变量数组m
        n=[0,0,0,0,0,0,0,0]  # 中间变量数组n
        for i in range(8):
            # 计算中间变量m[i]和n[i]
            m[i] = self.d[4]*(math.sin(t6[i])*(nx*math.cos(t1[i])+ny*math.sin(t1[i]))+math.cos(t6[i])*(ox*math.cos(t1[i])+oy* math.sin(t1[i])))-self.d[5]*(ax*math.cos(t1[i])+ay*math.sin(t1[i]))+px*math.cos(t1[i])+py*math.sin(t1[i])
            n[i] = pz-self.d[0]-az*self.d[5]+self.d[4]*(oz*math.cos(t6[i])+nz*math.sin(t6[i]))

        # 计算关节角3的值
        t3=[0,0,0,0,0,0,0,0]
        # 计算8个可能的theta3解
        t31 = math.acos((m[0]**2+n[0]**2-self.a[1]**2-self.a[2]**2)/(2*self.a[1]*self.a[2]))
        t32 = -math.acos((m[0]**2+n[0]**2-self.a[1]**2-self.a[2]**2)/(2*self.a[1]*self.a[2]))
        t33 = math.acos((m[2]**2+n[2]**2-self.a[1]**2-self.a[2]**2)/(2*self.a[1]*self.a[2]))
        t34 = -math.acos((m[2]**2+n[2]**2-self.a[1]**2-self.a[2]**2)/(2*self.a[1]*self.a[2]))
        t35 = math.acos((m[4]**2+n[4]**2-self.a[1]**2-self.a[2]**2)/(2*self.a[1]*self.a[2]))
        t36 = -math.acos((m[4]**2+n[4]**2-self.a[1]**2-self.a[2]**2)/(2*self.a[1]*self.a[2]))
        t37 = math.acos((m[6]**2+n[6]**2-self.a[1]**2-self.a[2]**2)/(2*self.a[1]*self.a[2]))
        t38 = -math.acos((m[6]**2+n[6]**2-self.a[1]**2-self.a[2]**2)/(2*self.a[1]*self.a[2]))
        t3 = [t31,t32,t33,t34,t35,t36,t37,t38]
        # 调整到[-π, π]范围内
        for i in range (8):
            if t3[i] > math.pi:
                t3[i] = t3[i] - 2 * math.pi
            elif t3[i] < -math.pi:
                t3[i] = t3[i] + 2 * math.pi

        # 计算关节角2
        t2=[0,0,0,0,0,0,0,0]  # 存储theta2的数组
        s2= [0,0,0,0,0,0,0,0] # 存储sin(theta2)的数组
        c2=[0,0,0,0,0,0,0,0]  # 存储cos(theta2)的数组
        for i in range (8):
            # 计算sin(theta2)和cos(theta2)
            s2[i]= ((self.a[2]*math.cos(t3[i])+self.a[1])* n[i]-self.a[2]*math.sin(t3[i])*m[i])/(self.a[1]**2+self.a[2]**2+2*self.a[1]*self.a[2]*math.cos(t3[i]))
            c2[i] =(m[i]+(self.a[2]*math.sin(t3[i])*s2[i]))/(self.a[2]*math.cos(t3[i])+self.a[1])
            # 通过反正切函数计算theta2
            t2[i] = math.atan2(s2[i],c2[i])
            # 调整到[-π, π]范围内
            if t2[i] > math.pi:
                t2[i] = t2[i] - 2 * math.pi
            elif t2[i] < -math.pi:
                t2[i] = t2[i] + 2 * math.pi

        # 计算关节角4
        t4=[0,0,0,0,0,0,0,0]  # 存储theta4的数组
        for i in range(8):
            # 计算theta4
            t4[i] = math.atan2(-math.sin(t6[i])*(nx*math.cos(t1[i])+ny*math.sin(t1[i]))-math.cos(t6[i])*(ox*math.cos(t1[i])+oy*math.sin(t1[i])),oz*math.cos(t6[i])+nz*math.sin(t6[i]))-t2[i]-t3[i]
            # 调整到[-π, π]范围内
            if t4[i] > math.pi:
                t4[i] = t4[i] - 2 * math.pi
            elif t4[i] < -math.pi:
                t4[i] = t4[i] + 2 * math.pi

        # 组装8组解
        a1 = [t1[0], t2[0], t3[0], t4[0], t5[0], t6[0]]  # 第一组解
        a2 = [t1[1], t2[1], t3[1], t4[1], t5[1], t6[1]]  # 第二组解
        a3 = [t1[2], t2[2], t3[2], t4[2], t5[2], t6[2]]  # 第三组解
        a4 = [t1[3], t2[3], t3[3], t4[3], t5[3], t6[3]]  # 第四组解
        a5 = [t1[4], t2[4], t3[4], t4[4], t5[4], t6[4]]  # 第五组解
        a6 = [t1[5], t2[5], t3[5], t4[5], t5[5], t6[5]]  # 第六组解
        a7 = [t1[6], t2[6], t3[6], t4[6], t5[6], t6[6]]  # 第七组解
        a8 = [t1[7], t2[7], t3[7], t4[7], t5[7], t6[7]]  # 第八组解

        # 将所有解组织成矩阵
<<<<<<< HEAD
        solution = np.asmatrix((a1,a2,a3,a4,a5,a6,a7,a8))
=======
        solution = np.mat((a1,a2,a3,a4,a5,a6,a7,a8))
>>>>>>> 9262b4f14817ac8e9d445ca81c2fd4d183d7b2d8
        
        # 找到与当前关节角度最接近的解
        closest_index = np.argmin(np.linalg.norm(solution - np.tile(curr_theta, (8,1)), axis=1))
        
        # 获取最优解
        opti_sol = solution[closest_index].tolist()[0]
        
        return opti_sol  # 返回最接近当前关节角度的解
    
    
    def Tmatrix_to_XYZrpy(self, T: np.matrix):
        # 将齐次变换矩阵转换为XYZ位置和RPY欧拉角
        # 计算欧拉角
        yaw = math.atan2(T[1, 0], T[0, 0])  # Yaw角（绕Z轴旋转）
        pitch = math.atan2(-T[2, 0], math.sqrt(T[0, 0]**2 + T[1, 0]**2))  # Pitch角（绕Y轴旋转）
        roll = math.atan2(T[2, 1], T[2, 2])  # Roll角（绕X轴旋转）
    
        # 提取位置坐标
        X = T[0, 3]  # x坐标
        Y = T[1, 3]  # y坐标
        Z = T[2, 3]  # z坐标
        
        XYZrpy = [X, Y, Z, roll, pitch, yaw]  # 组合位置和姿态
        
        return XYZrpy  # 返回[位置x, 位置y, 位置z, roll, pitch, yaw]
    
    def XYZrpy_to_Tmatrix(self, XYZrpy: list):
        # 将XYZ位置和RPY欧拉角转换为齐次变换矩阵
        # 预计算三角函数值，提高计算效率
        cos_roll = math.cos(XYZrpy[3])  # roll角的余弦值
        sin_roll = math.sin(XYZrpy[3])  # roll角的正弦值
        cos_pitch = math.cos(XYZrpy[4])  # pitch角的余弦值
        sin_pitch = math.sin(XYZrpy[4])  # pitch角的正弦值
        cos_yaw = math.cos(XYZrpy[5])  # yaw角的余弦值
        sin_yaw = math.sin(XYZrpy[5])  # yaw角的正弦值

        # 计算齐次变换矩阵
        T = np.array([
            # 第一行：[cos(yaw)*cos(pitch), cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll), cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll), x]
            [cos_yaw * cos_pitch, cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll, cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll, XYZrpy[0]],
            # 第二行：[sin(yaw)*cos(pitch), sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll), sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll), y]
            [sin_yaw * cos_pitch, sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll, sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll, XYZrpy[1]],
            # 第三行：[-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll), z]
            [-sin_pitch, cos_pitch * sin_roll, cos_pitch * cos_roll, XYZrpy[2]],
            # 第四行：[0, 0, 0, 1]
            [0, 0, 0, 1],
        ])

        return T  # 返回4x4齐次变换矩阵
    

    def rpy_to_rotvec(self, rpy: list):
     #将RPY(roll, pitch, yaw) 转换为UR10e的RX, RY, RZ（旋转向量）
     roll, pitch, yaw = rpy
    
     # 构造旋转矩阵（ZYX顺序）
     cos_roll = math.cos(roll)
     sin_roll = math.sin(roll)
     cos_pitch = math.cos(pitch)
     sin_pitch = math.sin(pitch)
     cos_yaw = math.cos(yaw)
     sin_yaw = math.sin(yaw)

     R = np.array([
        [cos_yaw * cos_pitch, cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll, cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll],
        [sin_yaw * cos_pitch, sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll, sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll],
        [-sin_pitch,cos_pitch * sin_roll,cos_pitch * cos_roll]
        ])

     # 从旋转矩阵 R 转换为旋转向量
     return self.rotation_matrix_to_rotvec(R)
    
    def rotvec_to_rpy(self, rotvec: list):
     #将 UR10e 的 RX, RY, RZ（旋转向量）转换为 RPY (roll, pitch, yaw)   
     R = self.rotvec_to_rotation_matrix(rotvec)
    
     # 从旋转矩阵提取 RPY (ZYX 顺序)
     if abs(R[2, 0]) < 1.0:
        pitch = -math.asin(R[2, 0])
        roll = math.atan2(R[2, 1] / math.cos(pitch), R[2, 2] / math.cos(pitch))
        yaw = math.atan2(R[1, 0] / math.cos(pitch), R[0, 0] / math.cos(pitch))
     else:
        # 万向节锁情况
        yaw = 0
        if R[2, 0] <= -1.0:
            pitch = math.pi / 2
            roll = math.atan2(R[0, 1], R[0, 2])
        else:
            pitch = -math.pi / 2
            roll = math.atan2(-R[0, 1], -R[0, 2])
    
     return [roll, pitch, yaw]
    
     
    def rotation_matrix_to_rotvec(self, R: np.ndarray):
     """
     将 3x3 旋转矩阵转换为旋转向量 [rx, ry, rz]
     """
     R = np.asarray(R)
     angle = math.acos((np.trace(R) - 1) / 2.0)
    
     if abs(angle) < 1e-10:
        return [0.0, 0.0, 0.0]
    
     if abs(angle - math.pi) < 1e-10:
        # 180度旋转，需特殊处理
        diag = np.diag(R)
        max_idx = np.argmax(diag)
        axis = np.zeros(3)
        axis[max_idx] = math.sqrt((diag[max_idx] + 1) / 2.0)
        others = [i for i in range(3) if i != max_idx]
        axis[others[0]] = R[others[0], max_idx] / (2 * axis[max_idx])
        axis[others[1]] = R[others[1], max_idx] / (2 * axis[max_idx])
     else:
        rx = R[2, 1] - R[1, 2]
        ry = R[0, 2] - R[2, 0]
        rz = R[1, 0] - R[0, 1]
        norm = math.sqrt(rx*rx + ry*ry + rz*rz)
        if norm < 1e-10:
            return [0.0, 0.0, 0.0]
        factor = angle / norm
        rx *= factor
        ry *= factor
        rz *= factor
        return [rx, ry, rz]
    
     # 处理180度情况的符号和归一化
     axis = axis / np.linalg.norm(axis)
     return (axis * angle).tolist()


    def rotvec_to_rotation_matrix(self, rotvec: list):
     """
     将旋转向量 [rx, ry, rz] 转换为 3x3 旋转矩阵
     使用 Rodrigues' rotation formula
     """
     rx, ry, rz = rotvec
     theta = math.sqrt(rx*rx + ry*ry + rz*rz)
    
     if theta < 1e-10:
        return np.eye(3)
    
     ux = rx / theta
     uy = ry / theta
     uz = rz / theta

     cos_t = math.cos(theta)
     sin_t = math.sin(theta)
     one_minus_cos = 1.0 - cos_t

     R = np.array([
        [cos_t + ux*ux*one_minus_cos,       ux*uy*one_minus_cos - uz*sin_t,   ux*uz*one_minus_cos + uy*sin_t],
        [uy*ux*one_minus_cos + uz*sin_t,    cos_t + uy*uy*one_minus_cos,      uy*uz*one_minus_cos - ux*sin_t],
        [uz*ux*one_minus_cos - uy*sin_t,    uz*uy*one_minus_cos + ux*sin_t,   cos_t + uz*uz*one_minus_cos]
     ])
    
     return R
    

    def xyzrpy_to_xyzrotvec(self, pose_rpy: list):
     """
     输入: [x, y, z, roll, pitch, yaw]
     输出: [x, y, z, rx, ry, rz]
     """
     x, y, z, roll, pitch, yaw = pose_rpy
     rotvec = self.rpy_to_rotvec([roll, pitch, yaw])
     return [x, y, z, rotvec[0], rotvec[1], rotvec[2]]

    def xyzrotvec_to_xyzrpy(self, pose_rotvec: list):
     """
     输入: [x, y, z, rx, ry, rz]
     输出: [x, y, z, roll, pitch, yaw]
     """
     x, y, z, rx, ry, rz = pose_rotvec
     rpy = self.rotvec_to_rpy([rx, ry, rz])
     return [x, y, z, rpy[0], rpy[1], rpy[2]]

    
<<<<<<< HEAD
   
=======
    def set_ik_iteration(self, iteration: int):
        # 设置逆运动学迭代次数
        self.ik_iteration = iteration  # 更新迭代次数
        print("逆运动学迭代次数设置为:", iteration)  # 打印确认信息
>>>>>>> 9262b4f14817ac8e9d445ca81c2fd4d183d7b2d8


if __name__ == "__main__":
    # 主程序入口，用于测试功能
    ur10eKine = UR10eKine()  # 创建UR10eKine类实例
    
    # 定义测试关节角度（弧度制）
    theta = [15/180*math.pi, -80/180*math.pi, -130/180*math.pi, 30/180*math.pi, 75/180*math.pi, -45/180*math.pi]
    print("theta: ", theta)  # 打印测试关节角度
    
    # 测试正运动学
    FK_result = ur10eKine.FK(theta)
    print(FK_result)  # 打印正运动学结果
    
    # 定义测试位置姿态
    XYZRPY = [647.9652874124298, -67.64341007055711, 166.16671027057987, -3.141592653589793, 0, 3.141592653589793]
<<<<<<< HEAD
    print("XYZRPY: ", XYZRPY)  # 打印测试位置姿态
=======
    print("XPZRPY: ", XYZRPY)  # 打印测试位置姿态
>>>>>>> 9262b4f14817ac8e9d445ca81c2fd4d183d7b2d8
    
    # 将位置姿态转换为齐次变换矩阵
    T = ur10eKine.XYZrpy_to_Tmatrix(XYZRPY)
    print("T: ", T)  # 打印变换矩阵

    
    print("测试逆运动学:")  # 打印提示信息
    IK_result = ur10eKine.IK(T, theta)
<<<<<<< HEAD
    print("1:",IK_result)
=======
    print(IK_result)
>>>>>>> 9262b4f14817ac8e9d445ca81c2fd4d183d7b2d8

    
    print("测试逆运动学得到的正运动学结果:")  # 打印提示信息
    FK_result = ur10eKine.FK(theta)
    print(FK_result)
    print("转为RPY是:")  # 打印提示信息
    FK_result_rpy = ur10eKine.Tmatrix_to_XYZrpy(FK_result)
<<<<<<< HEAD
    print("2:",FK_result_rpy)
=======
    print(FK_result_rpy)
>>>>>>> 9262b4f14817ac8e9d445ca81c2fd4d183d7b2d8

    rpy = [0, 0, math.pi]  # 绕Z轴转180度
    rotvec = ur10eKine.rpy_to_rotvec(rpy)
    print("RPY -> RotVec:", rotvec)  # 应接近 [0, 0, 3.14159]

    rotvec = [0, 0, math.pi]
    rpy_back = ur10eKine.rotvec_to_rpy(rotvec)
    print("RotVec -> RPY:", rpy_back)  # 应接近 [0, 0, 3.14159]

    pose_rpy = [500, 0, 300, 0, 0, math.pi]
    pose_rotvec = ur10eKine.xyzrpy_to_xyzrotvec(pose_rpy)
    print("XYZRPY -> XYZRXRYRZ:", pose_rotvec)