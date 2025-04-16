
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessStart

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

garage_navigator_pkg = get_package_share_directory("garage_navigator")

all_servers = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([garage_navigator_pkg, '/launch','/servers.launch.py']))
garage_navigation = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([garage_navigator_pkg, '/launch','/navigation_launch.py']))

def generate_launch_description():
        return LaunchDescription([
                all_servers,
                garage_navigation
        ])
