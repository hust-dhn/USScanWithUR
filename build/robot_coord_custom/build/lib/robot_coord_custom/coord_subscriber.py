#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from robot_coord_custom.msg import Coord6D  # 导入自定义消息

class CoordSubscriber(Node):
    def __init__(self):
        super().__init__('coord_subscriber')
        # 创建订阅者，订阅‘robot_coord‘话题，收到消息后调用listener_callback函数
        self.subscription = self.create_subscription(
            Coord6D,
            'robot_coord',
            self.listener_callback,
            10)  # 队列长度
        # 防止未使用变量警告（此行可省略，仅为代码规范）
        self.subscription
        self.get_logger().info('自定义坐标订阅节点已启动，等待数据...')

    def listener_callback(self, msg):
        """收到消息时的回调函数"""
        # 直接使用消息中的原始数据，无需任何转换
        self.get_logger().info(f'收到坐标:')
        self.get_logger().info(f'  位置 -> X: {msg.x:.3f} m, Y: {msg.y:.3f} m, Z: {msg.z:.3f} m')
        self.get_logger().info(f'  欧拉角 -> Rx: {msg.rx:.3f} rad, Ry: {msg.ry:.3f} rad, Rz: {msg.rz:.3f} rad')
        self.get_logger().info('---')

def main(args=None):
    rclpy.init(args=args)
    node = CoordSubscriber()
    rclpy.spin(node)  # 保持节点运行，等待消息到来
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()