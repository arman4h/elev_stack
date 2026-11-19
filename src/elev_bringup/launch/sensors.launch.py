"""Livox Mid-360 + ZED 2i + static TFs.

TF tree built here:
    odom --(ZED VIO)--> <zed_frame> --(static, inverse of base_to_zed)--> base_link
                                                      base_link --(static, 45deg pitch)--> livox_frame
Check with `ros2 run tf2_tools view_frames` that every frame has exactly ONE parent.
"""
import json
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from elev_bringup.tf_math import invert


def _static(frame, child, xyz, rpy):
    return Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--x', str(xyz[0]), '--y', str(xyz[1]), '--z', str(xyz[2]),
                   '--roll', str(rpy[0]), '--pitch', str(rpy[1]), '--yaw', str(rpy[2]),
                   '--frame-id', frame, '--child-frame-id', child])


def _setup(context):
    share = get_package_share_directory('elev_bringup')
    cfg = os.path.join(share, 'config')
    with open(os.path.join(cfg, 'mount.yaml')) as f:
        m = yaml.safe_load(f)
    zed_frame = LaunchConfiguration('zed_frame').perform(context)
    lidar_sn = LaunchConfiguration('lidar_sn').perform(context)

    with open(os.path.join(cfg, 'MID360_config.json')) as f:
        lidar_ip = json.load(f)['lidar_configs'][0]['ip']
    if 'XX' in lidar_ip:
        raise RuntimeError(
            'Lidar IP is still the placeholder in config/MID360_config.json. Run '
            'tools/set_lidar_ip.py <serial number> first (see VERIFY.md step 0).')

    L = m['base_to_livox']
    Z = m['base_to_zed']
    inv = invert(Z['x'], Z['y'], Z['z'], Z['roll'], Z['pitch'], Z['yaw'])  # zed -> base

    return [
        Node(package='livox_ros_driver2', executable='livox_ros_driver2_node',
             name='livox_lidar_publisher', output='screen',
             parameters=[{
                 'xfer_format': 0,          # 0 = sensor_msgs/PointCloud2 (x,y,z,intensity,tag,line)
                 'multi_topic': 0, 'data_src': 0, 'publish_freq': 10.0, 'output_data_type': 0,
                 'frame_id': 'livox_frame', 'lvx_file_path': '/tmp/livox.lvx',
                 'user_config_path': os.path.join(cfg, 'MID360_config.json'),
                 'cmdline_input_bd_code': lidar_sn}]),
        _static('base_link', 'livox_frame',
                (L['x'], L['y'], L['z']), (L['roll'], L['pitch'], L['yaw'])),
        _static(zed_frame, 'base_link', inv[:3], inv[3:]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('zed_wrapper'), 'launch', 'zed_camera.launch.py')),
            launch_arguments={
                'camera_model': 'zed2i', 'camera_name': 'zed',
                'publish_map_tf': 'false',
                'publish_urdf': 'false',  # zed URDF would give base_link a 2nd parent; static TFs above own the tree
                'ros_params_override_path': os.path.join(cfg, 'zed_override.yaml'),
            }.items()),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('zed_frame', default_value='zed_camera_link',
                              description='Frame the ZED wrapper publishes odom -> <frame> for'),
        DeclareLaunchArgument('lidar_sn', default_value='livox0000000001',
                              description='Mid-360 broadcast code (its serial number)'),
        OpaqueFunction(function=_setup),
    ])
