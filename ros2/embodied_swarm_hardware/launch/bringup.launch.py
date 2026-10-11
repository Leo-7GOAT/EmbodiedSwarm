from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def setup(context):
    share = Path(get_package_share_directory('embodied_swarm_hardware'))
    backend = LaunchConfiguration('backend').perform(context)
    config = (MoveItConfigsBuilder('panda', package_name='moveit_resources_panda_moveit_config')
              .robot_description(file_path=str(share / 'config/panda.urdf.xacro'), mappings={'backend': backend})
              .planning_pipelines(pipelines=['ompl']).to_moveit_configs())
    return [
        Node(package='tf2_ros', executable='static_transform_publisher', arguments=['0', '0', '0', '0', '0', '0', 'world', 'panda_link0']),
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[config.robot_description]),
        Node(package='controller_manager', executable='ros2_control_node', parameters=[config.robot_description, str(share / 'config/controllers.yaml')]),
        Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster', '--controller-manager-timeout', '30']),
        Node(package='controller_manager', executable='spawner', arguments=['panda_arm_controller', '--controller-manager-timeout', '30']),
        Node(package='moveit_ros_move_group', executable='move_group', parameters=[config.to_dict(), {'trajectory_execution.allowed_start_tolerance': 0.05}]),
    ]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('backend', default_value='mock', choices=['mock', 'mujoco']), OpaqueFunction(function=setup)])
