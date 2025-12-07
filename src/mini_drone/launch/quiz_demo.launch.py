"""
Quiz Demo Launch File

전체 퀴즈 데모 시스템을 실행합니다:
  1. cf_bridge - Crazyflie 텔레메트리 및 제어 브리지
  2. quiz_controller - 상태 머신 기반 퀴즈 컨트롤러

사용법:
  ros2 launch mini_drone quiz_demo.launch.py

  # Mini drone만 테스트 (ANAFI 없이)
  ros2 launch mini_drone quiz_demo.launch.py mini_only_mode:=true

  # 수직 모드
  ros2 launch mini_drone quiz_demo.launch.py vertical_mode:=True

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

    mini_only_mode_arg = DeclareLaunchArgument(
        'mini_only_mode',
        default_value='false',
        description='Test with Mini drone only (skip ANAFI). In DETECTING state, press 1/2 for manual answer.'
    )

    vertical_mode_arg = DeclareLaunchArgument(
        'vertical_mode',
        default_value='False',
        description='Vertical mode (True: vertical, False: horizontal)'
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
    # Quiz Controller Node
    quiz_controller_node = Node(
        package='mini_drone',
        executable='quiz_controller',
        name='quiz_controller',
        output='screen',
        # prefix='xterm -e',  # 별도 터미널에서 실행 (키보드 입력용)
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
            # Mini only mode
            'mini_only_mode': LaunchConfiguration('mini_only_mode'),
            'vertical_mode': LaunchConfiguration('vertical_mode'),
        }],
    )

    return LaunchDescription([
        # Launch arguments
        operation_timeout_arg,
        mini_home_z_arg,
        uri_arg,
        mini_only_mode_arg,
        vertical_mode_arg,
        # Nodes
        cf_bridge_node,
        quiz_controller_node,
    ])

