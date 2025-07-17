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
    from common import tracker_node, rsp_component, usb_driver_node, trajectory_node, detector_node, camera_node
    from launch_ros.actions import ComposableNodeContainer
    from launch.actions import Shutdown
    from launch import LaunchDescription


    container = ComposableNodeContainer(
        name='vision_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            rsp_component,
            camera_node,
            detector_node,
            usb_driver_node,
            tracker_node,
            trajectory_node,
        ],
        emulate_tty=True,
        on_exit=Shutdown(),
    )

    return LaunchDescription([
        container
    ])