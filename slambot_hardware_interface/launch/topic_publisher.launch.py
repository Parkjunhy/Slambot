import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    lidar_pkg_name = 'sllidar_ros2'
    
    lidar_launch_file = 'lidar.launch.py'

    lidar_pkg_dir = get_package_share_directory(lidar_pkg_name)
    lidar_launch_path = os.path.join(lidar_pkg_dir, 'launch', lidar_launch_file)

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch_path),
        launch_arguments={
            'serial_port': '/dev/ttyUSB0',  
            'frame_id': 'laser_link',      
            'serial_baudrate': '115200'     
        }.items()
    )

    imu_node = Node(
        package='slambot_hardware_interface',
        executable='imu_publish',
        name='bno08x_imu_node',
        output='screen'
    )

    driver_node = Node(
        package='slambot_hardware_interface',
        executable='driver',
        name='wheel_controller',
        output='screen'
    )

    return LaunchDescription([
        lidar_launch,
        imu_node,
        driver_node
    ])