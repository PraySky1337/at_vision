import sys
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('rm_bringup'), 'launch'))


node_params = os.path.join(
    get_package_share_directory('rm_bringup'), 'config', 'calibration_config.yaml')


def generate_launch_description():
    import common
    from launch_ros.actions import ComposableNodeContainer
    from launch.actions import Shutdown
    from launch import LaunchDescription

    rest_container = ComposableNodeContainer(
        name='rest_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            common.rsp_component,
            common.gimbal_node(node_params),
            common.camera_node(node_params),
        ],
        output='both',
        emulate_tty=True,
        on_exit=Shutdown()
    )

    return LaunchDescription([
        rest_container,
        common.calibration_node(node_params),
    ])