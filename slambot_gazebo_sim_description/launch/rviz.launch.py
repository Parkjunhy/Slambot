import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    
    # ---------------------------------------------------------
    # 1. 패키지 경로 및 파일 설정
    # ---------------------------------------------------------
    pkg_name = 'slambot_gazebo_sim_description'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Xacro(URDF) 파일 경로
    xacro_file = os.path.join(pkg_share, 'urdf', 'slambot_gazebo_sim.xacro')
    
    # RViz 설정 파일 경로 (기존 설정 유지)
    rviz_config_file = os.path.join(pkg_share, 'config', 'display.rviz')

    # ---------------------------------------------------------
    # 2. 로봇 모델 파싱 (Xacro -> XML)
    # ---------------------------------------------------------
    # 가제보 플러그인은 무시되고, 로봇의 형태와 링크 정보만 파싱됩니다.
    doc = xacro.process_file(xacro_file)
    robot_description = {'robot_description': doc.toxml()}

    # ---------------------------------------------------------
    # 3. 노드 설정
    # ---------------------------------------------------------
    
    # [핵심] Robot State Publisher
    # 역할: URDF를 읽어서 로봇의 각 링크(바퀴, 센서 등)의 위치 관계(TF)를 발행합니다.
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': False} # [중요] 실물 로봇이므로 False!
        ]
    )

    # [선택] Joint State Publisher
    # 역할: 바퀴 조인트 상태를 발행하여 TF 트리를 완성합니다.
    # (사용자님의 드라이버가 joint_states를 안 쏘는 경우 필수입니다)
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # RViz2 실행
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher,
        node_rviz
    ])