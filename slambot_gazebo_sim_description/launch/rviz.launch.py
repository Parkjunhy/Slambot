import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'slambot_gazebo_sim_description'
    pkg_share = get_package_share_directory(pkg_name)
    xacro_file = os.path.join(pkg_share, 'urdf', 'slambot_gazebo_sim.xacro')
    rviz_config_file = os.path.join(pkg_share, 'config', 'display.rviz')
    doc = xacro.process_file(xacro_file)
    robot_description = {'robot_description': doc.toxml()}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': False} 
        ]
    )

    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    ekf_node = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=['/home/jh/slambot_project/src/slambot_gazebo_sim_description/config/ekf.yaml'],
    )

    static_lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['-0.0785', '0', '0.16', '1.5708', '0', '0',
                'base_link', 'slambot_gazebo_sim/base_link/gpu_lidar'],
        name='static_lidar_tf'
    )

    static_imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1',
               'base_link', 'slambot_gazebo_sim/base_link/imu_sensor'],
    name='static_imu_tf'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher,
        node_rviz,
        ekf_node,
        static_imu_tf,
        static_lidar_tf,
    ])