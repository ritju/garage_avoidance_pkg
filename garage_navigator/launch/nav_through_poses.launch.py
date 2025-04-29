

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

garage_navigator_pkg = get_package_share_directory("garage_navigator")

params_file = os.path.join(garage_navigator_pkg, "params", "config.yaml")

def generate_launch_description():
        return LaunchDescription([
                Node(
                        package='garage_navigator',
                        executable='test_nav_through_poses',
                        name='nav_through_poses_client',
                        output='screen',
                        respawn_delay=2.0,
                        parameters=[params_file]
                ),
        ])







