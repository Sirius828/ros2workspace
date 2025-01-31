import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from Arm_Lib import Arm_Device
import math


class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        
        # 初始化机械臂设备
        self.arm = Arm_Device()  # 根据实际串口修改路径
        
        # 创建话题发布器
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        
        # 初始化标志位，控制是否读取角度
        self.read_enabled = True  # 默认允许读取关节角度
        
        # 服务客户端，用于监听执行节点的动作状态
        self.action_status_sub = self.create_subscription(
            JointState,
            '/arm_action_status',
            self.update_read_status,
            10
        )

        # 定时器：每隔 0.1 秒读取和发布关节状态
        self.timer = self.create_timer(0.1, self.publish_joint_states)

    def update_read_status(self, msg: JointState):
        """
        根据执行节点的动作状态控制是否暂停读取关节角度。
        """
        if 'executing' in msg.name:
            self.read_enabled = False
            self.get_logger().info("Pausing joint state reading due to active arm action.")
        else:
            self.read_enabled = True
            self.get_logger().info("Resuming joint state reading.")

    def publish_joint_states(self):
        if not self.read_enabled:
            return  # 如果标志位为 False，则跳过读取和发布

        # 创建 JointState 消息
        joint_states = JointState()
        joint_states.header.stamp = self.get_clock().now().to_msg()
        joint_states.name = [
            'arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4', 'arm_joint5', 'grip_joint'
        ]
        joint_states.position = []

        # 遍历关节 ID（1-6），读取角度并转换为弧度
        for joint_id in range(1, 7):
            angle = self.arm.Arm_serial_servo_read(joint_id)
            if angle is not None:
                radians = math.radians(angle)  # 转换为弧度
                # 将 0 到 pi 映射到 -pi/2 到 pi/2
                mapped_radians = radians - (math.pi / 2)
                joint_states.position.append(mapped_radians)
            else:
                # 如果读取失败，跳过该关节
                self.get_logger().warning(f"Failed to read angle for joint {joint_id}")
                joint_states.position.append(0.0)  # 设置为 0 或其他默认值

        # 发布关节状态
        self.publisher.publish(joint_states)
        # 调试
        # self.get_logger().info(f"Published joint states: {joint_states.position}")


def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()