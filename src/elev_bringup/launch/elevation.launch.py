"""Elevation mapper. Needs the lidar cloud (/livox/lidar) and TF odom -> base_link -> livox_frame
to be running already (driver, ZED and static TFs are set up separately)."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('elev_bringup'), 'config')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('backend', default_value='auto',
                              description='auto | cuda | cpu'),
        Node(package='elev_bringup', executable='elevation_mapper', output='screen',
             parameters=[os.path.join(cfg, 'mapper.yaml'),
                         {'backend': LaunchConfiguration('backend')}]),
        Node(package='rviz2', executable='rviz2', output='log',
             arguments=['-d', os.path.join(cfg, 'elevation.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
