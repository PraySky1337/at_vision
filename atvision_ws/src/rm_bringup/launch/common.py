import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.descriptions import ComposableNode

xacro_file = os.path.join(
    get_package_share_directory('rm_gimbal'), 'urdf', 'rm_gimbal.urdf.xacro')

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
    }],
    extra_arguments=[{'use_intra_process_comms': False}]
)

def camera_node(node_params):
    return ComposableNode(
        package='hik_camera',
        plugin='hik_camera::HikCameraNode',
        name='camera_node',
        parameters=[node_params],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

def armor_detector_node(node_params):
    return ComposableNode(
    package='armor_detector', 
    plugin='fyt::auto_aim::ArmorDetectorNode',
    name='armor_detector',
    parameters=[node_params],
    extra_arguments=[{'use_intra_process_comms': True}]
)

def armor_detector_ov_node(node_params): 
    return ComposableNode(
    package='armor_detector_ov', 
    plugin='rm_auto_aim::ArmorDetectorOVNode',
    name='armor_detector_ov',
    parameters=[node_params],
    extra_arguments=[{'use_intra_process_comms': True}])

def armor_solver_node(node_params):
    return ComposableNode(
    package='armor_solver',
    plugin='rm_auto_aim::ArmorSolverNode',
    name='armor_solver',
    parameters=[node_params],
)

def gimbal_node(node_params) :
    return ComposableNode(
    package='rm_gimbal',
    plugin='rm_gimbal::GimbalNode',
    name='rm_gimbal',
    parameters=[node_params],
    extra_arguments=[{'use_intra_process_comms': False}],
)

def pub_video_node(node_params) :
    return ComposableNode(
    package='pub_video',
    plugin='pub_video::PubVideoNode',
    name='pub_video',
    parameters=[node_params],
    extra_arguments=[{'use_intra_process_comms': True}],
)

def calibration_node(node_params) :
    return ComposableNode(
    package='rm_calibration',
    plugin='rm_calibration::CalibrationNode',
    name='calibration_node',
    parameters=[node_params],
    extra_arguments=[{'use_intra_process_comms': True}]
)
