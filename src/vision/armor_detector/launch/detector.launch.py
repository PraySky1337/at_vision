from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

node_params = os.path.join(
    get_package_share_directory('armor_detector'), 'config', 'config.yaml')

camera_node = Node(
    package='armor_detector',
    executable='armor_detector_node',
    name='armor_detector',  
    output='screen',
    emulate_tty=True,
    parameters=[node_params],
    arguments=['--ros-args', '--log-level', 'info'],
)


def generate_launch_description():
    return LaunchDescription([
        camera_node
    ])