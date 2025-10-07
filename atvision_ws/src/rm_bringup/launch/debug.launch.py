import sys
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('rm_auto_aim'), 'launch'))


node_params = os.path.join(
    get_package_share_directory('rm_auto_aim'), 'config', 'config.yaml')


def generate_launch_description():
    from common import armor_solver_node, rsp_component, usb_driver_node, armor_detector_node, camera_node, foxglove_node
    from launch_ros.actions import ComposableNodeContainer
    from launch.actions import Shutdown
    from launch import LaunchDescription

    rest_container = ComposableNodeContainer(
        name='rest_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            foxglove_node,
            rsp_component,
            usb_driver_node,
            armor_solver_node,
        ],
        output='both',
        emulate_tty=True,
        on_exit=Shutdown()
    )

    cam_detector_container = ComposableNodeContainer(
        name='cam_detector_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            camera_node,
            armor_detector_node
        ],
        output='both',
        emulate_tty=True,
        on_exit=Shutdown()
    )

    return LaunchDescription([
        rest_container,
        cam_detector_container,
    ])