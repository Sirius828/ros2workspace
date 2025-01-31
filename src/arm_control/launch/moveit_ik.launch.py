import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 使用 MoveItConfigsBuilder 加载配置
    moveit_config = ( 
        MoveItConfigsBuilder(
        robot_name="dummy",  # 机器人名称
        package_name="dummy_config",  # 配置包名称
        )
        .robot_description(file_path="config/dummy.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .moveit_cpp(
            file_path=get_package_share_directory("arm_control")
            + "/config/arm_config.yaml"
        )
        .to_moveit_configs()
    )

    # MoveIt Python API 节点
    moveit_ik_node = Node(
        package="arm_control",
        executable="moveit_ik_node",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # 姿态计算客户端
    target_pose_client = Node(
        package="arm_control",
        executable="target_pose_client",
        name="target_pose_client",
    )
    
    
    # 机器人状态发布节点
    joint_state_publisher = Node(
        package="arm_control",  # 修正包名
        executable="joint_state_publisher",
        name="joint_state_publisher",
    )
    
    
    # 机械臂控制节点
    arm_actuator = Node(
        package="arm_control",
        executable="arm_actuator",
        name="arm_actuator",
    )

    return LaunchDescription(
        [
            joint_state_publisher,
            target_pose_client,
            moveit_ik_node,
            arm_actuator,
        ]
    )