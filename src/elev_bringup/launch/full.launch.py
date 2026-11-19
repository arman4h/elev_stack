"""Real robot: Livox + ZED + TFs + elevation mapper (+ optional rviz2)."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('elev_bringup')
    inc = lambda f, args=None: IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', f)),
        launch_arguments=(args or {}).items())
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('backend', default_value='cuda'),
        inc('sensors.launch.py'),
        inc('mapping.launch.py', {'rviz': LaunchConfiguration('rviz'),
                                  'backend': LaunchConfiguration('backend')}),
    ])
