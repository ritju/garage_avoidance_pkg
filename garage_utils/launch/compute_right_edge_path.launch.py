import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    
    launch_description = LaunchDescription()

    # get pkg path
    garage_utils_pkg_path = get_package_share_directory('garage_utils')    

    # create launch configuration variables
    params_file_path = LaunchConfiguration('params_files', default=os.path.join(garage_utils_pkg_path, 'params', 'config.yaml'))

    log_level_arg = DeclareLaunchArgument('log_level', default_value='info', description='define compute_right_edge_path_action_server node log level')

    # compute_right_edge_path Node
    compute_right_edge_path_server_node = Node(
        executable='compute_right_edge_path_server_node',
        package='garage_utils',
        name='compute_right_edge_path_action_server',
        namespace='',
        output='screen',
        parameters=[params_file_path],
        arguments=['--ros-args', '--log-level', ['compute_right_edge_path_action_server:=', LaunchConfiguration('log_level')]]
    )
   
    launch_description.add_action(log_level_arg)
    launch_description.add_action(compute_right_edge_path_server_node)

    return launch_description
