import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
from scipy.spatial.transform import Rotation as R
import rtde_receive
import rtde_control
import math
import copy

class URros(Node):
    def __init__(self):
        super().__init__('ur_ros_node')
        
        # 初始化模式
        self.INIT_MODE = -1
        self.JP_MODE = 1   # Joint Position
        self.JV_MODE = 2   # Joint Velocity
        self.CP_MODE = 3   # Cartesian Position
        self.CV_MODE = 4   # Cartesian Velocity

        self.mode = -1     
        self.is_pubed = False  

        self.robot_ip = "192.168.253.102"
        self.rtde_c = rtde_control.RTDEControlInterface(self.robot_ip)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(self.robot_ip)

        self.jp_command = JointState()
        self.j_state = JointState()
        self.jv_command = JointState()
        self.cp_command = PoseStamped()
        self.cp_state = PoseStamped()
        self.cv_command = TwistStamped()
        self.cv_state = TwistStamped()
        self.tcpForce_state = WrenchStamped()

        # 获取初始状态
        self.get_logger().info(f"当前关节角: {self.rtde_r.getActualQ()}")
        self.get_logger().info(f"当前TCP位置: {self.rtde_r.getActualTCPPose()}")
        self.get_logger().info(f"当前TCP速度: {self.rtde_r.getTargetTCPSpeed()}")

        # 订阅者
        self.create_subscription(JointState, "ur10e/command/JointPosition", self.jp_callback, 10)
        self.create_subscription(JointState, "ur10e/command/JointVelocity", self.jv_callback, 10)
        self.create_subscription(PoseStamped, "ur10e/command/CartesianPose", self.cp_callback, 10)
        self.create_subscription(TwistStamped, "ur10e/command/CartesianVelocity", self.cv_callback, 10)

        # 发布者
        self.jsPub = self.create_publisher(JointState, "ur10e/state/JointState", 10)
        self.cpPub = self.create_publisher(PoseStamped, "ur10e/state/CartesianPose", 10)
        self.cvPub = self.create_publisher(TwistStamped, "ur10e/state/CartesianVelocity", 10)
        self.tcpforcePub = self.create_publisher(WrenchStamped, "ur10e/state/TCPForce", 10)

    def self_check(self):
        if self.rtde_c.isConnected():
            self.get_logger().info(" ------------Robot is Connected!!--------------")
        else:
            self.get_logger().error("ERROR! Robot is NOT connected ")
            self.get_logger().error("robot status code is %d", self.rtde_c.getRobotMode())

    # 回调函数
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

    # 发布状态
    def pub_of_state(self):
        stamp = self.get_clock().now().to_msg()

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
        print(f"------force------        {force}")
        self.is_pubed = True

    def robot_move_jp(self):
        velocity = 0.5
        acceleration = 0.5
        hz = 125
        dt = 1.0 / hz
        lookahead_time = 5 * dt
        gain = 300  # 伺服控制增益

        jp = self.jp_command.position
        self.rtde_c.servoJ(jp, velocity, acceleration, dt, lookahead_time, gain)

    def robot_move_jv(self):
        jv = self.jv_command.velocity
        self.rtde_c.speedJ(jv, 0.5, 0.002)

    def robot_move_cp(self):
        rotv = R.from_quat(
            [self.cp_command.pose.orientation.x, self.cp_command.pose.orientation.y, self.cp_command.pose.orientation.z,
             self.cp_command.pose.orientation.w]).as_rotvec()
        cp = [self.cp_command.pose.position.x, self.cp_command.pose.position.y, self.cp_command.pose.position.z,
              rotv[0], rotv[1], rotv[2]]
        self.rtde_c.moveL(cp, 0.5, 0.3)

    def robot_move_cv(self):
        cv = [self.cv_command.twist.linear.x, self.cv_command.twist.linear.y, self.cv_command.twist.linear.z,
              self.cv_command.twist.angular.x, self.cv_command.twist.angular.y, self.cv_command.twist.angular.z]
        self.rtde_c.speedL(cv, 0.25, 0.002)

    def __del__(self):
        self.rtde_c.stopScript()

    def jp_test(self):
        velocity = 0.5
        acceleration = 0.5
        gain = 300
        hz = 125
        rate = self.create_rate(hz)
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

            self.rtde_c.servoJ(jp_end, velocity, acceleration, 1.0 / 125, 5.0 / 125, gain)
            rate.sleep()
        return jp_end


def main():
    rclpy.init()
    ur_c = URros()
    rate = ur_c.create_rate(125)  # 125Hz

    ur_c.pub_of_state()

    while rclpy.ok():
        ur_c.pub_of_state()
        rate.sleep()

    ur_c.rtde_c.stopScript()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
