import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from nav2_common.launch import RewrittenYaml

"""
用于获取环境变量值
参数
env: 环境变量名称
default: 环境变量为赋值时使用的默认值
返回值
返回最终采用的值
"""
def get_environment_value(env, default):
    try:
        if env in os.environ:
            value = os.environ.get(env, default)
            print(f'get {env} value: {value} from environment')
            return value
        else:
            print(f"Using default {env} value: {default}.")
            return default
    except Exception as e:
        print(f'exception: {str(e)}')
        print(f"Please input {env} in environment")
        return default
def generate_launch_description():
    
    launch_description = LaunchDescription()

    # get pkg path
    garage_utils_pkg_path = get_package_share_directory('garage_utils')    

    # create launch configuration variables
    params_file_path = LaunchConfiguration('params_files', default=os.path.join(garage_utils_pkg_path, 'params', 'config.yaml'))

    log_level_arg = DeclareLaunchArgument('log_level', default_value='info', description='define compute_right_edge_path_action_server node log level')

    # 获取环境变量值
    off_set = get_environment_value("COMPUTE_RIGHT_EDGE_PATH_OFFSET", "0.8")
    resolution = get_environment_value("COMPUTE_RIGHT_EDGE_PATH_RESOLUTION", "1.0")
    allow_inverse = get_environment_value("COMPUTE_RIGHT_EDGE_PATH_ALLOW_INVERSE", "False")

    # 参数替换配置 - 确保值为字符串类型
    param_substitutions = {
        "offset": str(off_set),
        "resolution": str(resolution),
        "allow_inverse": str(allow_inverse).lower(),
    }    
    
    # 配置参数文件
    configured_params = RewrittenYaml(
        source_file=params_file_path,
        param_rewrites=param_substitutions,
        convert_types=True
    )    
    
    # compute_right_edge_path Node
    compute_right_edge_path_server_node = Node(
        executable='compute_right_edge_path_server_node',
        package='garage_utils',
        name='compute_right_edge_path_action_server',
        namespace='',
        output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', ['compute_right_edge_path_action_server:=', LaunchConfiguration('log_level')]]
    )
   
    launch_description.add_action(log_level_arg)
    launch_description.add_action(compute_right_edge_path_server_node)

    return launch_description
