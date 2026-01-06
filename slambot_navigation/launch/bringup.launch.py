import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. 패키지 경로 찾기 (FindPackageShare 사용)
    slambot_nav_pkg = FindPackageShare('slambot_navigation')
    
    # 2. 기본 파일 경로 설정 (PathJoinSubstitution 사용 - 이게 오픈소스 방식)
    default_map_file = PathJoinSubstitution([slambot_nav_pkg, 'maps', 'mape_new.yaml'])
    default_params_file = PathJoinSubstitution([slambot_nav_pkg, 'config', 'nav2_params.yaml'])
    
    # 3. 런치 파일 경로
    localization_launch_path = PathJoinSubstitution([slambot_nav_pkg, 'launch', 'localization.launch.py'])
    navigation_launch_path = PathJoinSubstitution([slambot_nav_pkg, 'launch', 'navigation.launch.py'])

    # 4. 실행 인수
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'map',
            default_value=default_map_file,
            description='Full path to map yaml file'
        ),

        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Full path to nav2 params'
        ),

        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),

        # 5. Localization 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch_path),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': params_file
            }.items()
        ),

        # 6. Navigation 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch_path),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': params_file
            }.items()
        ),
    ])