import os
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, Shutdown
from ament_index_python.packages import get_package_share_directory

node_params = os.path.join(
    get_package_share_directory('rm_bringup'), 'config', 'config.yaml')


def generate_launch_description():

    # Config file argument
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('energy_meter_solver'),
            'config',
            'params.yaml'
        ]),
        description='Path to parameter configuration file'
    )

    # Node
    energy_meter_solver_node = Node(
        package='energy_meter_solver',
        executable='energy_meter_solver_node',
        name='energy_meter_solver_node',
        output='screen'
    )

    # rm_gimbal node
    gimbal_container = ComposableNodeContainer(
        name='rest_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
            package='rm_gimbal',
            plugin='rm_gimbal::GimbalNode',
            name='rm_gimbal',
            parameters=[node_params],
            extra_arguments=[{'use_intra_process_comms': False}],
        )],
        output='both',
        emulate_tty=True,
        on_exit=Shutdown()
    )


    return LaunchDescription([
        config_arg,
        energy_meter_solver_node,
        gimbal_container,
    ])
