import rclpy
from rclpy.node import Node
from arm_interfaces.srv import ExecuteArmAction  # 使用自定义服务
from sensor_msgs.msg import JointState
from Arm_Lib import Arm_Device
import math
import time


class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')

        # 初始化机械臂设备
        self.arm = Arm_Device()

        # 初始化服务
        self.action_service = self.create_service(
            ExecuteArmAction,
            'execute_arm_action',
            self.execute_arm_action_callback
        )

        # 初始化动作状态发布
        self.action_status_pub = self.create_publisher(JointState, '/arm_action_status', 10)

        self.get_logger().info("ArmController service initialized.")

        # 初始化标志位
        self.is_executing = False
        
        # 定义比例系数和最小执行时间
        self.time_scale_factor = 0.0002  # 每度变化需要的时间（单位：秒/度）
        self.min_execution_time = 0.5  # 最小执行时间（单位：秒）

    def map_angle(self, angle_rad):
        """
        将角度从 [-π/2, π/2] 映射到 [0, π].
        """
        return (angle_rad + math.pi / 2) * (math.pi / (math.pi / 2 + math.pi / 2))

    # def calculate_execution_time(self, current_positions, target_positions):
    #     """
    #     根据目标与当前角度的差异计算执行时间。
    #     """
    #     max_angle_change = max(
    #         abs(math.degrees(target) - math.degrees(current))
    #         for current, target in zip(current_positions, target_positions)
    #     )
    #     # 动态计算时间，确保不低于最小时间
    #     execution_time = max(max_angle_change * self.time_scale_factor, self.min_execution_time)
        
    #     return execution_time
    
    def execute_arm_action_callback(self, request, response):
        """
        执行机械臂动作的服务回调。
        """
        if self.is_executing:
            response.success = False
            response.message = "Another action is already being executed."
            self.get_logger().warn(response.message)
            return response

        # 设置标志位，防止重复执行
        self.is_executing = True
        self.publish_action_status(True)  # 发布执行状态

        try:
            
            # 获取当前关节角度
            # current_positions = []
            # for joint_id in range(1, 7):
            #     angle = self.arm.Arm_serial_servo_read(joint_id)
            #     if angle is None:
            #         # self.get_logger().warning(f"Failed to read angle for joint {joint_id}. Using default value 0.0")
            #         current_positions.append(0.0)
            #     else:
            #         current_positions.append(math.radians(angle))

            
            # 获取目标关节位置并映射到 [0, π]
            joint_positions = [
                math.degrees(self.map_angle(pos)) for pos in request.joint_positions
            ]
            
            # 动态计算执行时间
            # execution_time = self.calculate_execution_time(current_positions, joint_positions)
            # self.get_logger().info("execution_time: %s" % execution_time)
            # 确保数组长度为6，补齐为0
            while len(joint_positions) < 6:
                joint_positions.append(0.0)

            # 写入角度并执行
            time.sleep(1.0)  # 等待一段时间,防止命令堵塞
            self.arm.Arm_serial_servo_write6_array(joint_positions, 1000)
            # time.sleep(request.execution_time)  # 等待执行完成
            # self.arm.Arm_serial_servo_write6_array(joint_positions, time=int(execution_time * 1000))
            # time.sleep(execution_time)  # 等待执行完成
            time.sleep(2.0)
            response.success = True
            response.message = f"Successfully executed action: {joint_positions}"
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Failed to execute arm action: {str(e)}"
            self.get_logger().error(response.message)
        finally:
            # 解除标志位
            self.is_executing = False
            self.publish_action_status(False)  # 发布非执行状态

        return response

    def publish_action_status(self, executing):
        """
        发布当前机械臂的动作状态。
        """
        status_msg = JointState()
        status_msg.header.stamp = self.get_clock().now().to_msg()
        status_msg.name = ['executing'] if executing else ['idle']
        self.action_status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()