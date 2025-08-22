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
    # pathWorldFile = 'worlds/slamworld.sdf'

    pathModelFile = os.path.join(get_package_share_directory(namePackage),modelFileRelativePath)
    robotDescription = xacro.process_file(pathModelFile).toxml()

    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'),
                                                               'launch', 'gz_sim.launch.py'))
    
    # gazeboLaunch = IncludeLaunchDescription(gazebo_rosPackageLaunch, 
    #                                         launch_arguments={'gz_args': f'-r {pathWorldFile}', 'on_exit_shutdown' : 'true'}.items())
    gazeboLaunch = IncludeLaunchDescription(gazebo_rosPackageLaunch, 
                                            launch_arguments={'gz_args': ['-r -v -v4 empty.sdf'], 'on_exit_shutdown' : 'true'}.items())

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
    )

    LaunchDescriptionObject = LaunchDescription()
    LaunchDescriptionObject.add_action(gazeboLaunch)
    LaunchDescriptionObject.add_action(spawnModelNodeGazebo)
    LaunchDescriptionObject.add_action(robot_state_publisher_node)
    LaunchDescriptionObject.add_action(joint_state_publisher_node)
    LaunchDescriptionObject.add_action(start_gazebo_ros_bridge_cmd)

    return LaunchDescriptionObject

#want home