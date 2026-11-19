"""No hardware: fake moving robot + synthetic Livox cloud + mapper (+ rviz2). Tests TF, 45deg
mount handling and the mapper end to end. Use backend:=cpu on a laptop without CUDA."""
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    share = get_package_share_directory('elev_bringup')
    m = yaml.safe_load(open(os.path.join(share, 'config', 'mount.yaml')))['base_to_livox']
    static = Node(package='tf2_ros', executable='static_transform_publisher', arguments=[
        '--x', str(m['x']), '--y', str(m['y']), '--z', str(m['z']),
        '--roll', str(m['roll']), '--pitch', str(m['pitch']), '--yaw', str(m['yaw']),
        '--frame-id', 'base_link', '--child-frame-id', 'livox_frame'])
    fake = Node(package='elev_bringup', executable='fake_lidar', output='screen')
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'mapping.launch.py')),
        launch_arguments={'rviz': LaunchConfiguration('rviz'),
                          'backend': LaunchConfiguration('backend')}.items())
    return [static, fake, mapping]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('backend', default_value='auto'),
        OpaqueFunction(function=_setup),
    ])
