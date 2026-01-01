from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    # Auto-detect rune type argument
    auto_detect_arg = DeclareLaunchArgument(
        'auto_detect_rune_type',
        default_value='false',
        choices=['true', 'false'],
        description='Auto-detect rune type (big/small) from motion pattern. If false, use rune_type from config file'
    )

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
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'auto_detect_type': LaunchConfiguration('auto_detect_rune_type')}
        ],
        remappings=[
            ('energy_meter_solver/cmd_gimbal', 'armor_solver/cmd_gimbal'),
        ]
    )

    return LaunchDescription([
        auto_detect_arg,
        config_arg,
        energy_meter_solver_node,
    ])
