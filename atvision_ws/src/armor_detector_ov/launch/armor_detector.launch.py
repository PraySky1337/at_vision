from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('armor_detector_ov'),
        'config',
        'config.yaml'
    )

    return LaunchDescription([
        Node(
            package='armor_detector_ov',         # 你的包名
            executable='armor_detector_ov_node',     # 已编译的可执行文件名
            name='armor_detector_ov',           # 节点名（与下方 YAML 对应）
            output='screen',
            parameters=[cfg],
        )
    ])
