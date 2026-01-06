import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # 1. 경로 및 변수 설정 (오픈소스 스타일)
    slambot_nav_pkg = FindPackageShare('slambot_navigation')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    
    # 2. 파라미터 재작성 (Sim Time 적용)
    # 맵 파일 경로는 여기서 섞지 않습니다! (경로 깨짐 방지)
    param_substitutions = {
        'use_sim_time': use_sim_time
    }
    
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True
    )
    
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([slambot_nav_pkg, 'maps', 'mape_new.yaml']),
            description='Full path to map yaml file'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([slambot_nav_pkg, 'config', 'nav2_params.yaml']),
            description='Full path to nav2 params'
        ),
        DeclareLaunchArgument('autostart', default_value='true'),

        # 3. Map Server (파라미터 독립 주입)
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                configured_params, 
                {'yaml_filename': map_yaml_file}
            ]
        ),
        
        # 4. AMCL
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[configured_params]
        ),
        
        # 5. Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': ['map_server', 'amcl']}]
        )
    ])