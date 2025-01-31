'''订阅：目标位姿'''
'''发布计算逆运动学请求和执行动作请求'''
'''发布：关节角度'''

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from arm_interfaces.srv import ComputeIK, ExecuteArmAction  # 自定义的服务

class TargetPoseClient(Node):
    def __init__(self):
        super().__init__('target_pose_client')

        # 初始化逆运动学计算服务客户端
        self.ik_client = self.create_client(ComputeIK, 'calculate_joint_angles')
        while not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Waiting for service "calculate_joint_angles"...')

        # 初始化执行机械臂动作服务客户端
        self.action_client = self.create_client(ExecuteArmAction, 'execute_arm_action')
        while not self.action_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Waiting for service "execute_arm_action"...')

        # 初始化目标位姿订阅
        self.target_pose_sub = self.create_subscription(
            PoseStamped,
            '/target_pose',
            self.process_target_pose,
            10
        )

        # 初始化关节角度发布
        self.joint_state_pub = self.create_publisher(JointState, '/calculated_joint_states', 10)

        # 初始化标志位
        self.processing = False  # 用于控制是否暂停读取

        self.get_logger().info("TargetPoseClient initialized.")

    def process_target_pose(self, pose: PoseStamped):
        # 检查标志位，防止重复处理
        if self.processing:
            self.get_logger().warning("Already processing a target pose. Ignoring new request.")
            return

        # 设置标志位，开始处理
        self.processing = True

        # 请求逆运动学计算
        request = ComputeIK.Request()
        request.target_pose = pose

        self.get_logger().info(f"Sending target pose to IK service: {pose.pose}")

        # 异步调用逆运动学服务
        future = self.ik_client.call_async(request)
        future.add_done_callback(self.handle_ik_response)

    def handle_ik_response(self, future):
        try:
            response = future.result()
            if response.success:
                # 发布关节状态
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = response.joint_state.name
                joint_state_msg.position = response.joint_state.position

                self.joint_state_pub.publish(joint_state_msg)
                self.get_logger().info(f"Published joint states: {dict(zip(response.joint_state.name, response.joint_state.position))}")

                # 调用执行节点的服务
                self.execute_arm_action(response.joint_state.name, response.joint_state.position)
            else:
                self.get_logger().error(f"Failed to calculate joint angles: {response.error_message}")
                self.processing = False  # 重置标志位
        except Exception as e:
            self.get_logger().error(f"IK service call failed: {e}")
            self.processing = False  # 重置标志位

    def execute_arm_action(self, joint_names, joint_positions):
        # 构造执行动作的服务请求
        action_request = ExecuteArmAction.Request()
        action_request.joint_names = joint_names
        action_request.joint_positions = joint_positions

        self.get_logger().info("Sending joint angles to execute arm action service.")

        # 异步调用动作执行服务
        future = self.action_client.call_async(action_request)
        future.add_done_callback(self.handle_action_response)

    def handle_action_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Arm action executed successfully.")
            else:
                self.get_logger().error("Failed to execute arm action.")
        except Exception as e:
            self.get_logger().error(f"Action service call failed: {e}")
        finally:
            # 重置标志位
            self.processing = False


def main(args=None):
    rclpy.init(args=args)
    node = TargetPoseClient()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()