
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

compute_right_edge_path_pkg = get_package_share_directory("garage_utils")
garage_navigator_pkg = get_package_share_directory("garage_navigator")
welt_params_file_path = os.path.join(garage_navigator_pkg, "params", "dsf_nav2_for_ad_4_exhibition.yaml")

def generate_launch_description():
        return LaunchDescription([
                IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([compute_right_edge_path_pkg, '/launch','/compute_right_edge_path.launch.py'])),
                Node(
                        executable='find_car_avoidance_point_node',
                        package='capella_multi_robots_info',
                        name='find_car_avoidance_point_node',
                        namespace='',
                        output='screen'
                ),
                Node(
                        executable='welt_model',
                        package='welt_model',
                        name='welt_model',
                        namespace='',
                        output='screen',
                        parameters=[welt_params_file_path],
                ),
        ])

