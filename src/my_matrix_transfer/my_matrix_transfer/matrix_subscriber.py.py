#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf2_ros import quaternion_matrix, translation_matrix

class MatrixSubscriber(Node):
    def __init__(self):
        super().__init__('matrix_subscriber')
        self.subscription = self.create_subscription(
            TransformStamped,
            'tf_matrix_topic',
            self.listener_callback,
            10)
        self.subscription  # 防止未使用变量警告
        self.get_logger().info('变换矩阵订阅节点已启动，等待数据...')

    def listener_callback(self, msg):
        self.get_logger().info(f'收到变换:')
        self.get_logger().info(f'  时间: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}')
        self.get_logger().info(f'  从 [{msg.header.frame_id}] 到 [{msg.child_frame_id}]')
        self.get_logger().info(f'  平移: [{msg.transform.translation.x:.2f}, {msg.transform.translation.y:.2f}, {msg.transform.translation.z:.2f}]')

        # ===== 可选：将消息重新组合为4x4矩阵 =====
        # 1. 从四元数获取3x3旋转矩阵
        quat = [msg.transform.rotation.x,
                msg.transform.rotation.y,
                msg.transform.rotation.z,
                msg.transform.rotation.w]
        rot_matrix = quaternion_matrix(quat) # 注意：这会得到一个4x4矩阵，其中包含旋转和位置

        # 2. 创建一个纯平移矩阵
        trans_matrix = translation_matrix([msg.transform.translation.x,
                                           msg.transform.translation.y,
                                           msg.transform.translation.z])

        # 3. 组合得到最终的4x4变换矩阵 (假设是平移后旋转，顺序根据你的需求调整)
        # 这里是一个简单的示例：T = translation * rotation
        T = np.dot(trans_matrix, rot_matrix)

        self.get_logger().info(f'  重构的4x4矩阵: \n{T}')
        self.get_logger().info('---')

def main(args=None):
    rclpy.init(args=args)
    node = MatrixSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()