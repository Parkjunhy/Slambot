import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    slambot_nav_pkg = FindPackageShare('slambot_navigation')
    
    default_map_file = PathJoinSubstitution([slambot_nav_pkg, 'maps', 'map_sim.yaml'])
    default_params_file = PathJoinSubstitution([slambot_nav_pkg, 'config', 'nav2_params_simul.yaml'])
    
    localization_launch_path = PathJoinSubstitution([slambot_nav_pkg, 'launch', 'localization.launch.py'])
    navigation_launch_path = PathJoinSubstitution([slambot_nav_pkg, 'launch', 'navigation.launch.py'])

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

        DeclareLaunchArgument('use_sim_time', default_value='true'), #simul
        DeclareLaunchArgument('autostart', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch_path),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': params_file
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch_path),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': params_file
            }.items()
        ),
    ])