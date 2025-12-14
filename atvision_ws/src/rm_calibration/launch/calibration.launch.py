from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('rm_calibration')
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'calibration_params.yaml'),
        description='Path to calibration config file'
    )
    
    # Calibration node
    calibration_node = Node(
        package='rm_calibration',
        executable='calibration_node',
        name='calibration_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file')
        ],
    )
    
    return LaunchDescription([
        config_file_arg,
        calibration_node
    ])