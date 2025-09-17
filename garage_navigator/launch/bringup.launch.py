
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

find_free_space_pkg = get_package_share_directory("find_free_space")
robot_avoidance_pkg = get_package_share_directory("robot_avoidance")
find_free_space_param_file_path = os.path.join(find_free_space_pkg, "params", "config.yaml")
print('find_free_space_param_file_path:', find_free_space_param_file_path)

garage_navigator_pkg = get_package_share_directory("garage_navigator")
welt_params_file_path = os.path.join(garage_navigator_pkg, "params", "dsf_nav2_for_ad_4_exhibition.yaml")

compute_right_edge_path = ExecuteProcess(
    cmd=['ros2', 'launch', 'garage_utils', 'compute_right_edge_path.launch.py'],
    output='screen'
)

welt = ExecuteProcess(
    cmd=['ros2', 'run', 'welt_model', 'welt_model'],
    output='screen'
)

find_free_space = Node(
                        package='find_free_space',
                        executable='find_parking_space',
                        name='find_free_space_action_server',
                        output='screen',
                        respawn_delay=2.0,
                        parameters=[find_free_space_param_file_path]
                )

find_free_space_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(find_free_space_pkg, 'launch', 'find_free_space.launch.py'))
    )


pub_rect_markers = Node(
                        package='garage_navigator',
                        executable='publisher_polygons_marker',
                        name='publisher_polygons_marker',
                        output='screen',
                        respawn_delay=2.0,
                )

garage_navigation = ExecuteProcess(
    cmd=['ros2', 'launch', 'garage_navigator', 'navigation_launch.py'],
    output='screen'
)

robot_avoidance = ExecuteProcess(
    cmd=['ros2', 'run', 'robot_avoidance', 'robot_avoidance',
         '--params-file', welt_params_file_path],
    output='screen'
)

robot_avoidance_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(robot_avoidance_pkg, 'launch', 'robot_avoidance.launch.py'))
    )

delay_garage_nav = RegisterEventHandler(
    event_handler=OnProcessStart(
        target_action=find_free_space,
        on_start=[garage_navigation]
    )
)

def generate_launch_description():
    return LaunchDescription([
        # robot_avoidance,
        robot_avoidance_launch,
        compute_right_edge_path,
        # find_free_space,
        find_free_space_launch,
        welt,
        pub_rect_markers,
        # delay_garage_nav,
        garage_navigation,
    ])
