import sys
import os
from launch import LaunchDescription
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('rm_bringup'), 'launch'))


node_params = os.path.join(
    get_package_share_directory('rm_bringup'), 'config', 'config.yaml')


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
            common.gimbal_node(node_params),
            common.armor_solver_node(node_params),
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
            common.camera_node(node_params),
            common.armor_detector_node(node_params),
        ],
        output='both',
        emulate_tty=True,
        on_exit=Shutdown(),
    )


    return LaunchDescription([
        common.rsp_node,
        rest_container,
        cam_detector_container
    ])
