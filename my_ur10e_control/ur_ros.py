import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import WrenchStamped

from scipy.spatial.transform import Rotation as R

import rtde_receive
import rtde_control

import math
import copy


class URros(object):
    def __init__(self):
        self.INIT_MODE = -1 #初始化
        self.JP_MODE = 1   #Joint Position
        self.JV_MODE = 2   #Joint elocity
        self.CP_MODE = 3   # 笛卡尔位置
        self.CV_MODE = 4   # 笛卡尔速度

        self.mode = -1     

        self.is_pubed = False  

        self.rate_init = 500   
        
        #初始化：rtde_c 和 rtde_r 分别是用于控制和接收数据的 RTDE 接口对象。
        self.robot_ip = "192.168.253.102"
        self.rtde_c = rtde_control.RTDEControlInterface(self.robot_ip)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(self.robot_ip)

        #状态变量：这些变量用于存储机器人的关节状态、TCP (Tool Center Point) 位置、速度和力等状态信息。
        self.jp_command = JointState()
        self.j_state = JointState()
        self.jv_command = JointState()
        self.cp_command = PoseStamped()
        self.cp_state = PoseStamped()
        self.cv_command = TwistStamped()
        self.cv_state = TwistStamped()
        self.tcpForce_state = WrenchStamped()

        # 读取并打印机器人的当前关节角度、TCP 位置和 TCP 速度。
        print(f"当前关节角: {self.rtde_r.getActualQ()}")
        print(f"当前TCP位置: {self.rtde_r.getActualTCPPose()}")
        print(f"当前TCP速度: {self.rtde_r.getTargetTCPSpeed()}")

        # 这些订阅者监听 ROS 主题，获取关节位置、速度和笛卡尔位置、速度的命令信息，并调用相应的回调函数。
        rospy.Subscriber("ur10e/command/JointPosition", JointState, self.jp_callback)
        rospy.Subscriber("ur10e/command/JointVelocity", JointState, self.jv_callback)
        rospy.Subscriber("ur10e/command/CartesianPose", PoseStamped, self.cp_callback)
        rospy.Subscriber("ur10e/command/CartesianVelocity", TwistStamped, self.cv_callback)

        # 这些发布者将机器人的当前状态发布到相应的 ROS 主题。
        self.jsPub = rospy.Publisher("ur10e/state/JointState", JointState, queue_size=1)
        self.cpPub = rospy.Publisher("ur10e/state/CartesianPose", PoseStamped, queue_size=1)
        self.cvPub = rospy.Publisher("ur10e/state/CartesianVelocity", TwistStamped, queue_size=1)
        self.tcpforcePub = rospy.Publisher("ur10e/state/TCPForce", WrenchStamped, queue_size=1)

    def self_check(self): #self_check 方法检查机器人是否连接，并根据连接状态打印相应的日志信息。
        if self.rtde_c.isConnected():
            rospy.loginfo(" ------------Robot is Connected!!--------------")
        else:
            rospy.logerr("ERROR! Robot is NOT connected ")
            rospy.logerr("robot status code is %d", self.rtde_c.getRobotMode())
        pass
    
    #这些回调函数接收来自 ROS 主题的命令信息，并设置相应的模式和命令变量。
    def jp_callback(self, msg):
        self.mode = self.JP_MODE
        self.jp_command = msg

    def jv_callback(self, msg):
        self.mode = self.JV_MODE
        self.jv_command = msg

    def cp_callback(self, msg):
        self.mode = self.CP_MODE
        self.cp_command = msg

    def cv_callback(self, msg):
        self.mode = self.CV_MODE
        self.cv_command = msg

    #发布状态：获取机器人的当前状态信息，并准备发布。
    def pub_of_state(self):

        stamp = rospy.Time().now()

        force = self.rtde_r.getActualTCPForce()
        jp = self.rtde_r.getActualQ()
        jv = self.rtde_r.getActualQd()

        cp = self.rtde_r.getActualTCPPose()
        quat = R.from_rotvec([cp[3], cp[4], cp[5]], degrees=False).as_quat()

        cv = self.rtde_r.getActualTCPSpeed()

        self.j_state.header.stamp = stamp
        self.j_state.position = jp
        self.j_state.velocity = jv

        self.cp_state.header.stamp = stamp
        self.cp_state.pose.position.x = cp[0]
        self.cp_state.pose.position.y = cp[1]
        self.cp_state.pose.position.z = cp[2]
        self.cp_state.pose.orientation.x = quat[0]
        self.cp_state.pose.orientation.y = quat[1]
        self.cp_state.pose.orientation.z = quat[2]
        self.cp_state.pose.orientation.w = quat[3]

        self.cv_state.header.stamp = stamp
        self.cv_state.twist.linear.x = cv[0]
        self.cv_state.twist.linear.y = cv[1]
        self.cv_state.twist.linear.z = cv[2]
        self.cv_state.twist.angular.x = cv[3]
        self.cv_state.twist.angular.y = cv[4]
        self.cv_state.twist.angular.z = cv[5]

        self.tcpForce_state.header.stamp = stamp
        self.tcpForce_state.wrench.force.x = force[0]
        self.tcpForce_state.wrench.force.y = force[1]
        self.tcpForce_state.wrench.force.z = force[2]
        self.tcpForce_state.wrench.torque.x = force[3]
        self.tcpForce_state.wrench.torque.y = force[4]
        self.tcpForce_state.wrench.torque.z = force[5]

        self.jsPub.publish(self.j_state)
        self.cpPub.publish(self.cp_state)
        self.cvPub.publish(self.cv_state)
        self.tcpforcePub.publish(self.tcpForce_state)
        self.is_pubed = True

        print(f"-----force-----    {force}")

        return 0

    # 这些方法分别控制机器人根据关节位置 (robot_move_jp)、关节速度 (robot_move_jv)、笛卡尔位置 (robot_move_cp) 和笛卡尔速度 (robot_move_cv) 进行运动。
    def robot_move_jp(self):
        velocity = 0.5
        acceleration = 0.5
        hz=125
        dt=1.0/hz
        lookahead_time=5*dt
        gain = 300 #伺服控制增益

        jp = self.jp_command.position
        # print(jp)
        # print("--------------------------")
        self.rtde_c.servoJ(jp, velocity, acceleration, dt,lookahead_time, gain)
        pass

    def robot_move_jv(self):
        jv = self.jv_command.velocity
        self.rtde_c.speedJ(jv, 0.5, 0.002)
        pass

    def robot_move_cp(self):
        rotv = R.from_quat(
            [self.cp_command.pose.orientation.x, self.cp_command.pose.orientation.y, self.cp_command.pose.orientation.z,
             self.cp_command.pose.orientation.w]).as_rotvec()
        cp = [self.cp_command.pose.position.x, self.cp_command.pose.position.y, self.cp_command.pose.position.z,
              rotv[0], rotv[1], rotv[2]]
        self.rtde_c.moveL(cp, 0.5, 0.3)
        return 0

    def robot_move_cv(self):
        cv = [self.cv_command.twist.linear.x, self.cv_command.twist.linear.y, self.cv_command.twist.linear.z,
              self.cv_command.twist.angular.x, self.cv_command.twist.angular.y, self.cv_command.twist.angular.z]
        self.rtde_c.speedL(cv, 0.25, 0.002)
        return 0

    #析构函数：在对象销毁时，调用 stopScript 方法停止机器人的所有运动。
    def __del__(self):
        self.rtde_c.stopScript()

    #关节位置测试:生成一个关节角度的测试轨迹，并控制机器人按照该轨迹运动。
    def jp_test(self):
        velocity = 0.5
        acceleration = 0.5
        gain = 300
        hz = 125
        rate = rospy.Rate(hz)
        # 生成轨迹 当前关节角开始
        jp_begin = [self.j_state.position[0], self.j_state.position[1], self.j_state.position[2],
                    self.j_state.position[3], self.j_state.position[4], self.j_state.position[5]]
        jp_end = [self.j_state.position[0], self.j_state.position[1], self.j_state.position[2],
                  self.j_state.position[3], self.j_state.position[4], self.j_state.position[5]]

        angle_range = 10.0 * math.pi / 180
        step_num = 2000
        for i in range(step_num):
            jp_end[3] = jp_begin[3] + (angle_range / 2000) * i
            jp_end[4] = jp_begin[4] + (angle_range / 2000) * i
            jp_end[5] = jp_begin[5] + (angle_range / 2000) * i

            print(jp_end)

            self.rtde_c.servoJ(jp_end, velocity, acceleration, 1.0 / 125, 5.0 / 125, gain)
            rate.sleep()
        return jp_end

#主函数：初始化 ROS 节点，创建 URros 类的实例，并进入主循环，根据模式调用相应的机器人运动方法。
def main():
    rospy.init_node("test")
    ur_c = URros()
    rate = rospy.Rate(125)

    ur_c.pub_of_state()

    # ur_c.jp_test()
    while not rospy.is_shutdown():
        # if ur_c.mode == ur_c.JP_MODE:
        #     ur_c.robot_move_jp()
        ur_c.pub_of_state()
        
        # print(ur_c.j_state)
        rate.sleep()
    ur_c.rtde_c.stopScript()
    return 0


if __name__ == "__main__":
    main()
