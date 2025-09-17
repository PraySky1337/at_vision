import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode

# launch_params = yaml.safe_load(open(os.path.join(
#     get_package_share_directory('rm_auto_aim'), 'config', 'launch_params.yaml')))


node_params = os.path.join(
    get_package_share_directory('rm_auto_aim'), 'config', 'config.yaml')

xacro_file = os.path.join(
    get_package_share_directory('rm_gimbal_description'), 'urdf', 'rm_gimbal.urdf.xacro')

robot_description = Command([
    'xacro ' + xacro_file
    # ' xyz:=' + xyz +
    # ' rpy:=' + rpy
])

rsp_component = ComposableNode(
    package='robot_state_publisher',
    plugin='robot_state_publisher::RobotStatePublisher',
    name='robot_state_publisher',
    parameters=[{
        'robot_description': robot_description,
        'publish_frequency': 1000.0
    }]
)

camera_node = ComposableNode(
    package='hik_camera',
    plugin='hik_camera::HikCameraNode',
    name='camera_node',
    parameters=[node_params],
    extra_arguments=[{'use_intra_process_comms': True}]
)

armor_detector_node = ComposableNode(
    package='armor_detector', 
    plugin='fyt::auto_aim::ArmorDetectorNode',
    name='armor_detector',
    parameters=[node_params],
    extra_arguments=[{'use_intra_process_comms': True}]
)

armor_solver_node = ComposableNode(
    package='armor_solver',
    plugin='fyt::auto_aim::ArmorSolverNode',
    name='armor_solver',
    parameters=[node_params],
)

usb_driver_node = ComposableNode(
    package='usb_driver',
    plugin='usb_driver::UsbDriver',
    name='usb_driver',
    parameters=[node_params],
    extra_arguments=[{'use_intra_process_comms': False}],
)