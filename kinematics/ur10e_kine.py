"""
UR10e机器人运动学
这是一个用于UR10e机器人运动学计算的Python类，实现正、逆运动学功能
"""

import math          
import numpy as np   
import time         
import rtde_control  
import rtde_receive 

rtde_c= None
rtde_r= None

class UR10eKine:
    def __init__(self, robot_ip='192.168.253.101'):
        global rtde_c,rtde_r
        self.a = [0, -0.6127, -0.57155, 0, 0, 0]           
        self.d = [0.1807, 0, 0, 0.17415, 0.11985, 0.11655]  
        self.alpha = [math.pi/2, 0, 0, math.pi/2, -math.pi/2, 0]  
        rtde_c = rtde_control.RTDEControlInterface(robot_ip)
        rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)
        self.T_end2tcp = np.matrix([
        [ 0,  1,  0,  0 ],
        [ 0,  0,  1,  0.07625 ],
        [ 1,  0,  0,  0.08900436787 ],
        [ 0,  0,  0,  1 ]
        ])
        
    def THT(self, Theta, A, D, Alpha):

        T = np.asmatrix((
            [math.cos(Theta), -math.sin(Theta)*math.cos(Alpha), math.sin(Alpha)*math.sin(Theta), A*math.cos(Theta)],
            [math.sin(Theta), math.cos(Theta)*math.cos(Alpha), -math.cos(Theta)*math.sin(Alpha), A*math.sin(Theta)],
            [0, math.sin(Alpha), math.cos(Alpha), D],
            [0, 0, 0, 1]
        ))
        return T 

    def FK(self, theta: list):
        T01 = self.THT(theta[0], self.a[0], self.d[0], self.alpha[0])  
        T12 = self.THT(theta[1], self.a[1], self.d[1], self.alpha[1])  
        T23 = self.THT(theta[2], self.a[2], self.d[2], self.alpha[2])  
        T34 = self.THT(theta[3], self.a[3], self.d[3], self.alpha[3])  
        T45 = self.THT(theta[4], self.a[4], self.d[4], self.alpha[4])  
        T56 = self.THT(theta[5], self.a[5], self.d[5], self.alpha[5])   
        T = np.matrix(T01 * T12 * T23 * T34 * T45 * T56 * self.T_end2tcp)
        return T  
 
    def IK(self, T: np.matrix, curr_theta: list):
        T = T @ np.linalg.inv(self.T_end2tcp)
        nx = T[0, 0]
        ny = T[1, 0]
        nz = T[2, 0]
        ox = T[0, 1]
        oy = T[1, 1]
        oz = T[2, 1]
        ax = T[0, 2]
        ay = T[1, 2]
        az = T[2, 2]
        px = T[0, 3]
        py = T[1, 3]
        pz = T[2, 3]
        
        # 求关节角1
        m = self.d[5] * ay - py
        n = ax * self.d[5] - px
        theta11 = math.atan2(m, n) - math.atan2(self.d[3], np.sqrt((m ** 2 + n ** 2 - (self.d[3]) ** 2)))
        theta12 = math.atan2(m, n) - math.atan2(self.d[3], -np.sqrt((m ** 2 + n ** 2 - (self.d[3]) ** 2)))
        t1=[theta11,theta11,theta11,theta11,theta12,theta12,theta12,theta12]
        for i in range (8):
            if t1[i] > math.pi:
                t1[i] = t1[i] - 2 * math.pi
            elif t1[i] < -math.pi:
                t1[i] = t1[i] + 2 * math.pi

        # 求关节角5
        theta51 = math.acos(ax * math.sin(theta11) - ay * math.cos(theta11))
        theta52 = -math.acos(ax * math.sin(theta11) - ay * math.cos(theta11))
        theta53 = math.acos(ax * math.sin(theta12) - ay * math.cos(theta12))
        theta54 = -math.acos(ax * math.sin(theta12) - ay * math.cos(theta12))
        t5= [theta51,theta51,theta52,theta52,theta53,theta53,theta54,theta54]
        for i in range (8):
            if t5[i] > math.pi:
                t5[i] = t5[i] - 2 * math.pi
            elif t5[i] < -math.pi:
                t5[i] = t5[i] + 2 * math.pi

        # 求关节角6
        t6= [0,0,0,0,0,0,0,0]
        for i in range (8):
            mm = nx * math.sin(t1[i]) - ny * math.cos(t1[i])
            nn = ox * math.sin(t1[i]) - oy * math.cos(t1[i])
            t6[i] = math.atan2(mm, nn) - math.atan2(math.sin(t5[i]), 0)
            if t6[i] > math.pi:
                t6[i] = t6[i] - 2 * math.pi
            elif t6[i] < -math.pi:
                t6[i] = t6[i] + 2 * math.pi

        # 求关节角3
        m=[0,0,0,0,0,0,0,0]
        n=[0,0,0,0,0,0,0,0]
        for i in range(8):
            m[i] = self.d[4]*(math.sin(t6[i])*(nx*math.cos(t1[i])+ny*math.sin(t1[i]))+math.cos(t6[i])*(ox*math.cos(t1[i])+oy* math.sin(t1[i])))-self.d[5]*(ax*math.cos(t1[i])+ay*math.sin(t1[i]))+px*math.cos(t1[i])+py*math.sin(t1[i])
            n[i] = pz-self.d[0]-az*self.d[5]+self.d[4]*(oz*math.cos(t6[i])+nz*math.sin(t6[i]))
        t3=[0,0,0,0,0,0,0,0]
        t31 = math.acos((m[0]**2+n[0]**2-self.a[1]**2-self.a[2]**2)/(2*self.a[1]*self.a[2]))
        t32 = -math.acos((m[0]**2+n[0]**2-self.a[1]**2-self.a[2]**2)/(2*self.a[1]*self.a[2]))
        t33 = math.acos((m[2]**2+n[2]**2-self.a[1]**2-self.a[2]**2)/(2*self.a[1]*self.a[2]))
        t34 = -math.acos((m[2]**2+n[2]**2-self.a[1]**2-self.a[2]**2)/(2*self.a[1]*self.a[2]))
        t35 = math.acos((m[4]**2+n[4]**2-self.a[1]**2-self.a[2]**2)/(2*self.a[1]*self.a[2]))
        t36 = -math.acos((m[4]**2+n[4]**2-self.a[1]**2-self.a[2]**2)/(2*self.a[1]*self.a[2]))
        t37 = math.acos((m[6]**2+n[6]**2-self.a[1]**2-self.a[2]**2)/(2*self.a[1]*self.a[2]))
        t38 = -math.acos((m[6]**2+n[6]**2-self.a[1]**2-self.a[2]**2)/(2*self.a[1]*self.a[2]))
        t3 = [t31,t32,t33,t34,t35,t36,t37,t38]
        for i in range (8):
            if t3[i] > math.pi:
                t3[i] = t3[i] - 2 * math.pi
            elif t3[i] < -math.pi:
                t3[i] = t3[i] + 2 * math.pi

        # 求关节角2
        t2=[0,0,0,0,0,0,0,0]
        s2= [0,0,0,0,0,0,0,0]
        c2=[0,0,0,0,0,0,0,0]
        for i in range (8):
            s2[i]= ((self.a[2]*math.cos(t3[i])+self.a[1])* n[i]-self.a[2]*math.sin(t3[i])*m[i])/(self.a[1]**2+self.a[2]**2+2*self.a[1]*self.a[2]*math.cos(t3[i]))
            c2[i] =(m[i]+(self.a[2]*math.sin(t3[i])*s2[i]))/(self.a[2]*math.cos(t3[i])+self.a[1])
            t2[i] = math.atan2(s2[i],c2[i])
            if t2[i] > math.pi:
                t2[i] = t2[i] - 2 * math.pi
            elif t2[i] < -math.pi:
                t2[i] = t2[i] + 2 * math.pi

        # 求关节角4
        t4=[0,0,0,0,0,0,0,0]
        for i in range(8):
            t4[i] = math.atan2(-math.sin(t6[i])*(nx*math.cos(t1[i])+ny*math.sin(t1[i]))-math.cos(t6[i])*(ox*math.cos(t1[i])+oy*math.sin(t1[i])),oz*math.cos(t6[i])+nz*math.sin(t6[i]))-t2[i]-t3[i]
            if t4[i] > math.pi:
                t4[i] = t4[i] - 2 * math.pi
            elif t4[i] < -math.pi:
                t4[i] = t4[i] + 2 * math.pi

        a1 = [t1[0], t2[0], t3[0], t4[0], t5[0], t6[0]]
        a2 = [t1[1], t2[1], t3[1], t4[1], t5[1], t6[1]]
        a3 = [t1[2], t2[2], t3[2], t4[2], t5[2], t6[2]]
        a4 = [t1[3], t2[3], t3[3], t4[3], t5[3], t6[3]]
        a5 = [t1[4], t2[4], t3[4], t4[4], t5[4], t6[4]]
        a6 = [t1[5], t2[5], t3[5], t4[5], t5[5], t6[5]]
        a7 = [t1[6], t2[6], t3[6], t4[6], t5[6], t6[6]]
        a8 = [t1[7], t2[7], t3[7], t4[7], t5[7], t6[7]]

        solution = np.asmatrix((a1,a2,a3,a4,a5,a6,a7,a8))
        closest_index = np.argmin(np.linalg.norm(solution - np.tile(curr_theta, (8,1)), axis=1))
        opti_sol = solution[closest_index].tolist()[0]
        
        return opti_sol
    
    def Tmatrix_to_XYZRXRYRZ(self, T_matrix):
     x = T_matrix[0, 3]
     y = T_matrix[1, 3]
     z = T_matrix[2, 3]
     R = T_matrix[:3, :3]
     trace = R[0, 0] + R[1, 1] + R[2, 2]
     cos_angle = (trace - 1) / 2
     angle = math.acos(cos_angle) 
     sin_angle = math.sin(angle) 
     factor = 1 / (2 * sin_angle)
     rx = factor * (R[2, 1] - R[1, 2])
     ry = factor * (R[0, 2] - R[2, 0])
     rz = factor * (R[1, 0] - R[0, 1])
     return [x, y, z, rx, ry, rz]

    def XYZRXRYRZ_to_Tmatrix(self, xyzrxryrz):
     x, y, z, rx, ry, rz = xyzrxryrz
     angle = math.sqrt(rx**2 + ry**2 + rz**2)
     ux = rx / angle
     uy = ry / angle
     uz = rz / angle
     cos_a = math.cos(angle)
     sin_a = math.sin(angle)
     one_minus_cos_a = 1 - cos_a
     R = np.matrix([
            [cos_a + ux**2 * one_minus_cos_a, ux * uy * one_minus_cos_a - uz * sin_a, ux * uz * one_minus_cos_a + uy * sin_a],
            [uy * ux * one_minus_cos_a + uz * sin_a, cos_a + uy**2 * one_minus_cos_a, uy * uz * one_minus_cos_a - ux * sin_a],
            [uz * ux * one_minus_cos_a - uy * sin_a, uz * uy * one_minus_cos_a + ux * sin_a, cos_a + uz**2 * one_minus_cos_a]
        ])
     T = np.matrix([
        [R[0, 0], R[0, 1], R[0, 2], x],
        [R[1, 0], R[1, 1], R[1, 2], y],
        [R[2, 0], R[2, 1], R[2, 2], z],
        [0, 0, 0, 1]
        ])
     return T
    
def main():
    global rtde_c,rtde_r
    ur10eKine = UR10eKine() 

    # Test1: 正运动学测试
    theta_jiaodu = [90, -90, 90, -90, 90, -90]
    theta = [theta_jiaodu[i]/180*math.pi for i in range(6)]
    FK_result = ur10eKine.FK(theta)
    FK_result_rmv_offset = FK_result @ np.linalg.inv(ur10eKine.T_end2tcp)
    start_time = time.time()
    target_pose = ur10eKine.Tmatrix_to_XYZRXRYRZ(FK_result_rmv_offset)
    end_time = time.time()
    print(f"转换计算时间: {end_time - start_time} 秒")
    print("目标位姿:\n",target_pose)

    velocity = 0.1 
    acceleration = 0.1 
    rtde_c.moveJ(theta, velocity, acceleration)
    tcp_pose = rtde_r.getActualTCPPose()


    print("\n对比:\n",np.array(tcp_pose) - np.array(target_pose))
    threshold = 1e-3  
    error_distance = np.linalg.norm(np.array(tcp_pose) - np.array(target_pose))
    print(f"欧几里得误差距离: {error_distance}")
    is_close = error_distance < threshold
    print(f"欧几里得距离是否接近? {is_close} ")
    rmse = np.sqrt(np.mean((np.array(tcp_pose) - np.array(target_pose))**2))
    print(f"均方根误差: {rmse}")
    is_close = rmse < threshold
    print(f"均方根是否接近? {is_close} ")

    # Test2: 逆运动学测试
    IK_result = ur10eKine.IK(FK_result, theta)
    print("逆运动学结果:\n",IK_result)

    # Test3: IK -> MoveL(z-0.01m)
    T = FK_result.copy()
    z_sum = 0
    while True:
        T[2, 3] -= 0.001
        IK_result = ur10eKine.IK(T, theta)
        print("测试逆运动学:\n",IK_result)
        rtde_c.moveJ(IK_result, velocity, acceleration)
        theta = rtde_r.getActualQ()
        print("theta: \n", theta)
        z_sum += 1
        if z_sum >= 20:
            break
 
if __name__ == "__main__":
    main()