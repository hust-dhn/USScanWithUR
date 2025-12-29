#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rclpy
from rclpy.node import Node
from robot_coord_custom.msg import Coord6D

class CoordPublisher(Node):
    def __init__(self):
        super().__init__('coord_publisher')  # 节点名
        # 创建发布者，消息类型为 Coord6D，话题名为 ‘robot_coord‘，队列长度10
        self.publisher_ = self.create_publisher(Coord6D, 'robot_coord', 10)
        
        # 设置定时器，每秒发布一次 (周期0.008s)
        timer_period = 0.008
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # 初始化一个计数变量，用来模拟变化的数据
        self.i = 0
        self.get_logger().info('自定义坐标发布节点已启动！')

    def timer_callback(self):
        """定时器回调函数，每次触发时构建并发布消息"""
        # 1. 创建自定义消息的实例
        msg = Coord6D()
        
        # 2. 填充消息数据
        # 这里模拟数据变化，你可以替换为从传感器或其他程序获得的真实数据
        msg.x = 1.0 + self.i * 0.1
        msg.y = 2.0
        msg.z = 0.5
        # 欧拉角 (以弧度为单位)
        msg.rx = 0.0
        msg.ry = 0.0
        msg.rz = self.i * 0.1  # 绕Z轴每次增加0.1弧度
        
        # 3. 发布消息
        self.publisher_.publish(msg)
        
        # 4. 在终端打印日志，方便调试
        self.get_logger().info(f'发布坐标: [{msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f}, {msg.rx:.2f}, {msg.ry:.2f}, {msg.rz:.2f}]')
        
        # 5. 计数器加一
        self.i += 1

def main(args=None):
    rclpy.init(args=args)          # 初始化ROS2 Python客户端库
    node = CoordPublisher()        # 创建节点实例
    rclpy.spin(node)               # 保持节点运行，等待回调触发
    node.destroy_node()            # 关闭节点
    rclpy.shutdown()               # 关闭ROS2 Python客户端库

if __name__ == '__main__':
    main()