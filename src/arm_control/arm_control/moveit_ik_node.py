'''IK 服务节点'''
'''接收目标位姿，计算关节角度'''


import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy
from moveit.planning import TrajectoryExecutionManager
from geometry_msgs.msg import PoseStamped, Vector3, Quaternion
from arm_interfaces.srv import ComputeIK  # 使用自定义的 ComputeIK 服务
from moveit_msgs.msg import RobotTrajectory, Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive

class MoveItIKService(Node):
    def __init__(self):
        super().__init__('moveit_ik_service')
        self.moveit = MoveItPy(node_name="moveit_py")
        self.arm = self.moveit.get_planning_component("arm")

        if not self.arm:
            self.get_logger().error("FAILED to get planning component for 'arm'")
            return

        # 创建服务
        self.srv = self.create_service(ComputeIK, 'calculate_joint_angles', self.calculate_joint_angles_callback)
        self.get_logger().info("Service 'calculate_joint_angles' ready.")

    def calculate_joint_angles_callback(self, request, response):
        target_pose = request.target_pose  # 从请求中提取目标位姿

        # 创建约束
        constraints = Constraints()

        # 添加位置约束
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = "base_link"  # 设置参考坐标系
        position_constraint.link_name = "arm_link5"  # 修改为实际的末端工具链
        position_constraint.target_point_offset = Vector3(x=0.0, y=0.0, z=0.0)

        # 定义一个合理大小的约束区域
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.01, 0.01, 0.01]  # 设置约束区域（宽度、高度、深度）

        position_constraint.constraint_region.primitives.append(primitive)
        position_constraint.constraint_region.primitive_poses.append(target_pose.pose)  # 使用目标位姿
        position_constraint.weight = 1.0
        constraints.position_constraints.append(position_constraint)

        # 添加姿态约束
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = "base_link"
        orientation_constraint.link_name = "arm_link5"
        orientation_constraint.orientation = target_pose.pose.orientation  # 目标姿态
        orientation_constraint.absolute_x_axis_tolerance = 0.01  # 合理的姿态容忍度
        orientation_constraint.absolute_y_axis_tolerance = 0.01
        orientation_constraint.absolute_z_axis_tolerance = 0.01
        orientation_constraint.weight = 1.0  # 提高姿态约束的重要性
        constraints.orientation_constraints.append(orientation_constraint)
        # 设置运动规划约束
        self.arm.set_start_state_to_current_state()
        success = self.arm.set_goal_state(motion_plan_constraints=[constraints])
        if not success:
            self.get_logger().error("Failed to set goal state.")
            response.success = False
            response.error_message = "Failed to set goal state."
            return response

        # 执行规划
        plan_result = self.arm.plan()
        if plan_result and plan_result.trajectory.get_robot_trajectory_msg().joint_trajectory.points:
            final_point = plan_result.trajectory.get_robot_trajectory_msg().joint_trajectory.points[-1]
            joint_names = plan_result.trajectory.get_robot_trajectory_msg().joint_trajectory.joint_names
            joint_positions = final_point.positions

            # 填充响应
            response.success = True
            response.joint_state.name = joint_names
            response.joint_state.position = joint_positions
            response.error_message = ""
            self.get_logger().info(f"Joint positions calculated: {dict(zip(joint_names, joint_positions))}")
        else:
            response.success = False
            response.error_message = "Failed to calculate joint positions."
            self.get_logger().error("Failed to calculate joint positions.")

        return response


def main(args=None):
    rclpy.init(args=args)
    node = MoveItIKService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()