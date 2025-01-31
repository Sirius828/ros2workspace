import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.spatial.transform import Rotation as R

class TargetPosePublisher(Node):
    def __init__(self):
        super().__init__('target_pose_publisher')
        self.publisher = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.timer = self.create_timer(2.0, self.publish_target_pose)  # 每2秒发布一次

    def adjust_orientation(self, quat):
        """
        对四元数进行修正：
        1. 绕Z轴顺时针旋转90度。
        2. 绕Y轴顺时针旋转90度。
        """
        rotation = R.from_quat([quat['x'], quat['y'], quat['z'], quat['w']])
        
        # 组合旋转：Z轴逆时针90° 和 Y轴顺时针90°
        correction_z = R.from_euler('z', np.pi / 2)  # Z轴顺时针旋转90度
        correction_y = R.from_euler('y', np.pi / 2)  # Y轴顺时针旋转90度
        corrected_rotation = rotation * correction_z * correction_y
        
        corrected_quat = corrected_rotation.as_quat()  # 转换为四元数
        return {
            'x': round(corrected_quat[0], 7),  # 保留7位小数
            'y': round(corrected_quat[1], 7),  # 保留7位小数
            'z': round(corrected_quat[2], 7),  # 保留7位小数
            'w': round(corrected_quat[3], 7)   # 保留7位小数
        }

    def publish_target_pose(self):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'base_link'
        pose.pose.position.x = 0.0041600
        pose.pose.position.y = 0.2289500
        pose.pose.position.z = 0.2583100

        # 原始四元数
        original_quat = {
            'x': 0.0,
            'y': 0.0,
            'z': 0.0,
            'w': 1.0  # 默认位姿，机械爪朝天
        }

        # 修正后的四元数
        corrected_quat = self.adjust_orientation(original_quat)
        pose.pose.orientation.x = corrected_quat['x']
        pose.pose.orientation.y = corrected_quat['y']
        pose.pose.orientation.z = corrected_quat['z']
        pose.pose.orientation.w = corrected_quat['w']

        self.publisher.publish(pose)

        # 高精度日志输出
        self.get_logger().info(
            f'Published corrected target pose:\n'
            f'Position:\n'
            f'  x = {pose.pose.position.x:.7f}\n'
            f'  y = {pose.pose.position.y:.7f}\n'
            f'  z = {pose.pose.position.z:.7f}\n'
            f'Orientation:\n'
            f'  x = {pose.pose.orientation.x:.7f}\n'
            f'  y = {pose.pose.orientation.y:.7f}\n'
            f'  z = {pose.pose.orientation.z:.7f}\n'
            f'  w = {pose.pose.orientation.w:.7f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = TargetPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()