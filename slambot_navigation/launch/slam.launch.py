import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg = 'slambot_navigation'
    share = get_package_share_directory(pkg)

    slam_params = os.path.join(share, 'config', 'slam_toolbox_parameters.yaml')

    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true'
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    ld = LaunchDescription()
    ld.add_action(use_sim_time)
    ld.add_action(slam_node)

    return ld