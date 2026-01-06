import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    robotXacroName = 'slambot_gazebo_sim'
    namePackage = 'slambot_gazebo_sim_description'
    modelFileRelativePath = 'urdf/slambot_gazebo_sim.xacro'
    pathWorldFile = os.path.join(get_package_share_directory(namePackage), 'worlds', 'mapworld.sdf')

    pathModelFile = os.path.join(get_package_share_directory(namePackage),modelFileRelativePath)
    robotDescription = xacro.process_file(pathModelFile).toxml()
    rviz_config_file = os.path.join(get_package_share_directory(namePackage), 'config', 'display.rviz')

    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'),
                                                               'launch', 'gz_sim.launch.py'))
    
    gazeboLaunch = IncludeLaunchDescription(gazebo_rosPackageLaunch, 
                                            launch_arguments={'gz_args': f'-r -v4 {pathWorldFile}', 'on_exit_shutdown' : 'true'}.items())  
    # gazeboLaunch = IncludeLaunchDescription(gazebo_rosPackageLaunch, 
    #                                         launch_arguments={'gz_args': ['-r -v -v4 empty.sdf'], 'on_exit_shutdown' : 'true'}.items())
    
    spawnModelNodeGazebo = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robotXacroName,
            '-topic', 'robot_description',
            '-x','0','-y','0','-z', str(0.01),
        ],
        output='screen'
    )
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robotDescription, 'use_sim_time' : True}],
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    bridge_params = os.path.join(
    get_package_share_directory(namePackage),
    'parameters',
    'bridge_parameters.yaml',
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
        '--ros-args',
        '-p',
        f'config_file:={bridge_params}',
        ],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
    )

    static_lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1',
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

    ekf_node = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=['/home/jh/slambot_project/src/slambot_gazebo_sim_description/config/ekf.yaml'],
    # 필요하면 remap 가능
    # remappings=[('/odometry/filtered', '/odometry/filtered')]
    )


    LaunchDescriptionObject = LaunchDescription()
    LaunchDescriptionObject.add_action(gazeboLaunch)
    LaunchDescriptionObject.add_action(spawnModelNodeGazebo)
    LaunchDescriptionObject.add_action(robot_state_publisher_node)
    LaunchDescriptionObject.add_action(joint_state_publisher_node)
    LaunchDescriptionObject.add_action(start_gazebo_ros_bridge_cmd)
    LaunchDescriptionObject.add_action(rviz_node)
    LaunchDescriptionObject.add_action(static_lidar_tf)
    LaunchDescriptionObject.add_action(static_imu_tf)
    LaunchDescriptionObject.add_action(ekf_node)

    return LaunchDescriptionObject