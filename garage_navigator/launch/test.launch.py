
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

params_file = os.path.join(garage_navigator_pkg, "params", "config.yaml")

def generate_launch_description():
        return LaunchDescription([
                Node(
                        package='garage_navigator',
                        executable='test_action',
                        name='test_garage_action_client',
                        output='screen',
                        respawn_delay=2.0,
                        parameters=[params_file]
                ),
        ])
