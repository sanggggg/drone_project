"""
Quiz Demo Launch File

전체 퀴즈 데모 시스템을 실행합니다:
  1. cf_bridge - Crazyflie 텔레메트리 및 제어 브리지
  2. ai_deck_camera - AI-Deck 카메라 스트리밍
  3. quiz_controller - 상태 머신 기반 퀴즈 컨트롤러

사용법:
  ros2 launch mini_drone quiz_demo.launch.py

  # 파라미터 변경 예시
  ros2 launch mini_drone quiz_demo.launch.py operation_timeout_min:=3.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Launch arguments
    operation_timeout_arg = DeclareLaunchArgument(
        'operation_timeout_min',
        default_value='5.0',
        description='Operation timeout in minutes'
    )

    mini_home_z_arg = DeclareLaunchArgument(
        'mini_home_z',
        default_value='0.4',
        description='Mini drone home altitude (m)'
    )

    uri_arg = DeclareLaunchArgument(
        'uri',
        default_value='radio://0/80/2M/E7E7E7E711',
        description='Crazyflie radio URI'
    )

    # Crazyflie Bridge Node
    cf_bridge_node = Node(
        package='mini_drone',
        executable='cf_bridge',
        name='cf_bridge',
        output='screen',
        parameters=[{
            'uri': LaunchConfiguration('uri'),
            'period_ms': 100,
            'publish_rate_hz': 20.0,
            'use_state_estimate': True,
            'hl_takeoff_duration_s': 2.0,
            'hl_land_duration_s': 2.0,
            'hl_goto_duration_s': 5.1,
        }],
    )

    # AI-Deck Camera Node (optional - comment out if not using camera)
    ai_deck_camera_node = Node(
        package='mini_drone',
        executable='ai_deck_camera',
        name='ai_deck_camera',
        output='screen',
        parameters=[{
            'host': '192.168.4.1',
            'port': 5000,
        }],
    )

    # Quiz Controller Node
    quiz_controller_node = Node(
        package='mini_drone',
        executable='quiz_controller',
        name='quiz_controller',
        output='screen',
        prefix='xterm -e',  # 별도 터미널에서 실행 (키보드 입력용)
        parameters=[{
            # Mini drone 홈 위치
            'mini_home_x': 0.0,
            'mini_home_y': 0.0,
            'mini_home_z': LaunchConfiguration('mini_home_z'),
            'mini_home_yaw_deg': 0.0,
            # ANAFI 홈 위치
            'anafi_home_x': 0.0,
            'anafi_home_y': 0.0,
            'anafi_home_z': 0.6,
            'anafi_home_yaw_deg': 0.0,
            # 타이밍
            'takeoff_duration_s': 2.0,
            'land_duration_s': 2.0,
            'command_settle_time_s': 0.2,
            'operation_timeout_min': LaunchConfiguration('operation_timeout_min'),
            # 홈 도달 판정
            'home_position_tolerance_m': 0.1,
            'home_yaw_tolerance_deg': 15.0,
        }],
    )

    return LaunchDescription([
        # Launch arguments
        operation_timeout_arg,
        mini_home_z_arg,
        uri_arg,
        # Nodes
        cf_bridge_node,
        ai_deck_camera_node,
        quiz_controller_node,
    ])

