"""
Quiz Demo Launch File

전체 퀴즈 데모 시스템을 실행합니다:
  1. anafi - ANAFI ROS2 bridge (Olympe SDK)
  2. cf_bridge - Crazyflie 텔레메트리 및 제어 브리지
  3. yolo_detection - YOLO 기반 객체 인식 노드
  4. quiz_controller - 상태 머신 기반 퀴즈 컨트롤러

사용법:
  # 기본 실행 (team namespace 없음)
  ros2 launch mini_drone quiz_demo.launch.py

  # Team namespace 지정 (멀티 로봇)
  ros2 launch mini_drone quiz_demo.launch.py team:=team9
  ros2 launch mini_drone quiz_demo.launch.py team:=team11

  # 두 팀 동시 실행 예시 (별도 터미널에서)
  # Terminal 1:
  ros2 launch mini_drone quiz_demo.launch.py team:=team9 uri:='radio://0/80/2M/E7E7E7E709' anafi_ip:='192.168.53.1'
  # Terminal 2:
  ros2 launch mini_drone quiz_demo.launch.py team:=team11 uri:='radio://0/90/2M/E7E7E7E711' anafi_ip:='192.168.54.1'

  # Mini drone만 테스트 (ANAFI 없이)
  ros2 launch mini_drone quiz_demo.launch.py mini_only_mode:=true

  # 수직 모드
  ros2 launch mini_drone quiz_demo.launch.py vertical_mode:=True

  # ANAFI 파라미터 변경 예시
  ros2 launch mini_drone quiz_demo.launch.py anafi_ip:='192.168.42.1' anafi_model:='ai'

  # YOLO 파라미터 변경 예시
  ros2 launch mini_drone quiz_demo.launch.py yolo_model:='yolov8n.pt' yolo_confidence:=0.5

  # Crazyflie URI 변경 예시
  ros2 launch mini_drone quiz_demo.launch.py uri:='radio://0/80/2M/E7E7E7E709'

Topic Structure (with team namespace, e.g., team=team9):
  /team9/anafi/...              - ANAFI drone topics (camera/image, drone/state, etc.)
  /team9/cf/...                 - Crazyflie drone topics (odom, hl/takeoff, etc.)
  /team9/quiz/...               - Quiz controller topics (state, command, answer, beep)
  /team9/yolo/...               - YOLO detection outputs (image, detections, etc.)

Without team namespace (team=''):
  /anafi/...                    - ANAFI drone topics
  /cf/...                       - Crazyflie drone topics
  /quiz/...                     - Quiz controller topics
  /yolo/...                     - YOLO detection outputs
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ==========================================================================
    # Team Namespace Argument
    # ==========================================================================
    team_arg = DeclareLaunchArgument(
        'team',
        default_value='',
        description='Team namespace (e.g., team9, team11). Empty for no team namespace.'
    )

    # ==========================================================================
    # ANAFI Launch Arguments
    # ==========================================================================
    anafi_namespace_arg = DeclareLaunchArgument(
        'anafi_namespace',
        default_value='anafi',
        description='Namespace for ANAFI drone (within team namespace)'
    )

    anafi_ip_arg = DeclareLaunchArgument(
        'anafi_ip',
        default_value='192.168.53.1',  # SkyController: '192.168.53.1', Direct: '192.168.42.1', Sphinx: '10.202.0.1'
        description='IP address of the ANAFI device'
    )

    anafi_model_arg = DeclareLaunchArgument(
        'anafi_model',
        default_value='ai',  # {'4k', 'thermal', 'usa', 'ai'}
        description='Model of the ANAFI drone'
    )

    # ==========================================================================
    # Crazyflie Launch Arguments
    # ==========================================================================
    uri_arg = DeclareLaunchArgument(
        'uri',
        default_value='radio://0/80/2M/E7E7E7E711',
        description='Crazyflie radio URI'
    )

    cf_period_ms_arg = DeclareLaunchArgument(
        'cf_period_ms',
        default_value='100',
        description='Crazyflie telemetry logging period (ms)'
    )

    cf_publish_rate_hz_arg = DeclareLaunchArgument(
        'cf_publish_rate_hz',
        default_value='20.0',
        description='Crazyflie ROS publish rate (Hz)'
    )

    cf_hl_goto_duration_s_arg = DeclareLaunchArgument(
        'cf_hl_goto_duration_s',
        default_value='5.1',
        description='Crazyflie high-level goto duration (s)'
    )

    # ==========================================================================
    # YOLO Detection Launch Arguments
    # ==========================================================================
    yolo_model_arg = DeclareLaunchArgument(
        'yolo_model',
        default_value='yolo11m_fhd.engine',
        description='YOLO model path or name'
    )

    yolo_device_arg = DeclareLaunchArgument(
        'yolo_device',
        default_value='cuda',
        description='Torch device hint: "", "cpu", "cuda"'
    )

    yolo_confidence_arg = DeclareLaunchArgument(
        'yolo_confidence',
        default_value='0.25',
        description='Detection confidence threshold'
    )

    yolo_inference_rate_arg = DeclareLaunchArgument(
        'yolo_inference_rate',
        default_value='10',
        description='Max inference rate in Hz'
    )

    yolo_publish_compressed_arg = DeclareLaunchArgument(
        'yolo_publish_compressed',
        default_value='true',
        description='Whether to publish compressed image'
    )

    yolo_ocr_enabled_arg = DeclareLaunchArgument(
        'yolo_ocr_enabled',
        default_value='true',
        description='Enable OCR on detected regions'
    )

    # ==========================================================================
    # Quiz Controller Launch Arguments
    # ==========================================================================
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

    mini_only_mode_arg = DeclareLaunchArgument(
        'mini_only_mode',
        default_value='false',
        description='Test with Mini drone only (skip ANAFI). In DETECTING state, press 1/2 for manual answer.'
    )

    vertical_mode_arg = DeclareLaunchArgument(
        'vertical_mode',
        default_value='true',
        description='Vertical mode (True: vertical, False: horizontal)'
    )

    # ==========================================================================
    # ANAFI Node
    # ==========================================================================
    anafi_config = os.path.join(
        get_package_share_directory('anafi_ros_nodes'),
        'params.yaml'
    )

    anafi_node = Node(
        package='anafi_ros_nodes',
        namespace=LaunchConfiguration('anafi_namespace'),
        executable='anafi',
        name='anafi',
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'INFO'],
        parameters=[anafi_config,
            {'drone/model': LaunchConfiguration('anafi_model')},
            {'device/ip': LaunchConfiguration('anafi_ip')}
        ]
    )

    # ==========================================================================
    # Crazyflie Bridge Node (namespace 'cf')
    # ==========================================================================
    cf_bridge_node = Node(
        package='mini_drone',
        namespace='cf',
        executable='cf_bridge',
        name='cf_bridge',
        output='screen',
        parameters=[{
            'uri': LaunchConfiguration('uri'),
            'period_ms': LaunchConfiguration('cf_period_ms'),
            'publish_rate_hz': LaunchConfiguration('cf_publish_rate_hz'),
            'use_state_estimate': True,
            'hl_takeoff_duration_s': 2.0,
            'hl_land_duration_s': 2.0,
            'hl_goto_duration_s': LaunchConfiguration('cf_hl_goto_duration_s'),
        }],
    )

    # ==========================================================================
    # YOLO Detection Node
    # At team root level (no extra namespace) for proper cross-namespace access
    # All topics are relative to team namespace:
    #   - anafi/camera/image → /<team>/anafi/camera/image
    #   - quiz/* → /<team>/quiz/*
    #   - yolo/* → /<team>/yolo/* (published topics)
    # ==========================================================================
    yolo_detection_node = Node(
        package='anafi_ros_nodes',
        executable='yolo_detection',
        name='yolo_detection',
        output='screen',
        parameters=[{
            'model': LaunchConfiguration('yolo_model'),
            'device': LaunchConfiguration('yolo_device'),
            'confidence': LaunchConfiguration('yolo_confidence'),
            'inference_rate': LaunchConfiguration('yolo_inference_rate'),
            'publish_compressed': LaunchConfiguration('yolo_publish_compressed'),
            'camera_topic': 'anafi/camera/image',  # → /<team>/anafi/camera/image
            'iou_threshold': 0.45,
            'max_detections': 100,
            'classes': [62, 63],  # TV/monitor classes
            'ocr_enabled': LaunchConfiguration('yolo_ocr_enabled'),
            # 'ocr_classes': [],  # 빈 리스트는 launch에서 지원 안됨 - 노드 기본값 사용
            'ocr_buffer_size': 10,
            'ocr_timeout_sec': 4.0,
            'ocr_confidence_threshold': 90.0,
        }],
    )

    # ==========================================================================
    # Quiz Controller Node
    # No namespace - sits at team root level to access sibling namespaces
    # Topics: quiz/*, cf/*, anafi/* (all relative to team namespace)
    # ==========================================================================
    quiz_controller_node = Node(
        package='mini_drone',
        executable='quiz_controller',
        name='quiz_controller',
        output='screen',
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
        # No remappings needed - relative paths work within team namespace:
        # quiz/answer → /<team>/quiz/answer
        # cf/odom → /<team>/cf/odom
        # anafi/drone/state → /<team>/anafi/drone/state
    )

    # ==========================================================================
    # Group all nodes under team namespace
    # ==========================================================================
    # PushRosNamespace will prepend the team namespace to all nodes in the group
    # If team is empty, nodes will be at root level
    team_group = GroupAction([
        PushRosNamespace(LaunchConfiguration('team')),
        anafi_node,
        cf_bridge_node,
        yolo_detection_node,
        quiz_controller_node,
    ])

    return LaunchDescription([
        # ==========================================================================
        # Launch Arguments
        # ==========================================================================
        # Team namespace
        team_arg,
        # ANAFI arguments
        anafi_namespace_arg,
        anafi_ip_arg,
        anafi_model_arg,
        # Crazyflie arguments
        uri_arg,
        cf_period_ms_arg,
        cf_publish_rate_hz_arg,
        cf_hl_goto_duration_s_arg,
        # YOLO arguments
        yolo_model_arg,
        yolo_device_arg,
        yolo_confidence_arg,
        yolo_inference_rate_arg,
        yolo_publish_compressed_arg,
        yolo_ocr_enabled_arg,
        # Quiz controller arguments
        operation_timeout_arg,
        mini_home_z_arg,
        mini_only_mode_arg,
        vertical_mode_arg,

        # ==========================================================================
        # Node Group (with team namespace)
        # ==========================================================================
        team_group,
    ])
