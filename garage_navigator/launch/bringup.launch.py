
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


robot_avoidance_params_file_path = os.path.join(garage_navigator_pkg, "params", "dsf_nav2_for_ad_4_exhibition.yaml")
robot_avoidance = Node(
                        executable='robot_avoidance',
                        package='robot_avoidance',
                        name='robot_avoidance',
                        namespace='',
                        output='screen',
                        parameters=[robot_avoidance_params_file_path],
                )

def generate_launch_description():
        return LaunchDescription([
                robot_avoidance,
                all_servers,
                garage_navigation,
        ])
