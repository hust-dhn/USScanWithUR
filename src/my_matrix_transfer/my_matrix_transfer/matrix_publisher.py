#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf2_ros import quaternion_from_matrix # 用于从矩阵提取四元数

class MatrixPublisher(Node):
    def __init__(self):
        super().__init__('matrix_publisher')
        self.publisher_ = self.create_publisher(TransformStamped, 'tf_matrix_topic', 10)
        timer_period = 0.1  # 每0.1秒发布一次
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('变换矩阵发布节点已启动...')

    def timer_callback(self):
        # ===== 1. 这里是你的4x4变换矩阵 =====
        # 示例：一个单位矩阵，你可以替换成你的实际矩阵
        my_4x4_matrix = np.eye(4)
        # 假设我们给它一些简单的平移和旋转变化
        my_4x4_matrix[0, 3] = 1.0  # 沿X轴平移1米
        # ... 其他矩阵赋值操作
        # =================================

        # ===== 2. 将矩阵转换为ROS消息 =====
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'     # 父坐标系
        msg.child_frame_id = 'robot_base' # 子坐标系

        # 提取平移部分 (矩阵的最后一列的前三个元素)
        msg.transform.translation.x = float(my_4x4_matrix[0, 3])
        msg.transform.translation.y = float(my_4x4_matrix[1, 3])
        msg.transform.translation.z = float(my_4x4_matrix[2, 3])

        # 提取旋转部分 (将3x3旋转矩阵转换为四元数)
        rotation_matrix = my_4x4_matrix[:3, :3]
        quat = quaternion_from_matrix(my_4x4_matrix) # 这是从4x4矩阵获取四元数的便捷方法
        msg.transform.rotation.x = quat[0]
        msg.transform.rotation.y = quat[1]
        msg.transform.rotation.z = quat[2]
        msg.transform.rotation.w = quat[3]

        # ===== 3. 发布消息 =====
        self.publisher_.publish(msg)
        self.get_logger().info(f'发布变换: 从 [{msg.header.frame_id}] 到 [{msg.child_frame_id}]')

def main(args=None):
    rclpy.init(args=args)
    node = MatrixPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()