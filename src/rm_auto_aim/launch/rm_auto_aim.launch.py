from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_directory

node_params = os.path.join(
    get_package_share_directory('rm_auto_aim'), 'config', 'config.yaml')

robot_description = os.path.join(
    get_package_share_directory('rm_gimbal_description'), 'urdf', 'rm_gimbal.urdf.xacro')

camera_node = Node(
    package='rm_auto_aim',
    executable='rm_auto_aim',
    output='screen',
    emulate_tty=True,
    parameters=[node_params],
    arguments=['--ros-args', '--log-level', 'info'],
)

foxglove_bridge_node = Node(
    package='foxglove_bridge',       
    executable='foxglove_bridge',
    name='foxglove_bridge',
    output='screen',
    parameters=[{
        'port': 8765
    }],
)

robot_description = Command(['xacro ', robot_description])

robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': robot_description,
                 'publish_frequency': 1000.0}]
)


def generate_launch_description():
    return LaunchDescription([
        robot_state_publisher,
        camera_node,
        foxglove_bridge_node
    ])