from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ip = LaunchConfiguration('ip')
    camera_name = LaunchConfiguration('camera_name')
    frame_id = LaunchConfiguration('frame_id')
    image_topic = LaunchConfiguration('image_topic')
    gscam_config = LaunchConfiguration('gscam_config')
    enable_state = LaunchConfiguration('enable_state')
    enable_follower = LaunchConfiguration('enable_follower')

    return LaunchDescription([
        DeclareLaunchArgument('ip', default_value='192.168.53.1'),
        DeclareLaunchArgument('camera_name', default_value='anafi_ai'),
        DeclareLaunchArgument('frame_id', default_value='anafi_ai_camera'),
        DeclareLaunchArgument('image_topic', default_value='/anafi_ai/image_raw'),
        DeclareLaunchArgument('enable_state', default_value='true'),
        DeclareLaunchArgument('enable_follower', default_value='true'),
        DeclareLaunchArgument(
            'gscam_config',
            default_value=(
                'rtspsrc location=rtsp://192.168.53.1/live protocols=tcp latency=100 ! '
                'rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! '
                'video/x-raw,format=BGR ! appsink name=appsink sync=false drop=true max-buffers=1'
            ),
        ),

        # 카메라 (gscam)
        Node(
            package='gscam',
            executable='gscam_node',
            name='gscam_node',
            output='screen',
            parameters=[{
                'gscam_config': gscam_config,
                'camera_name': camera_name,
                'frame_id': frame_id,
            }],
            remappings=[('/image_raw', image_topic)],
        ),

        # 상태/제어 브릿지
        Node(
            package='anafi_ai_bridge',
            executable='state_bridge',
            name='anafi_state_bridge',
            output='screen',
            parameters=[{
                'ip': ip,
                'base_frame': 'anafi/base_link',
                'map_frame': 'map',
                'publish_rate_hz': 30.0,
                'angles_in_degrees': True,
                'fill_pose_altitude': True,
                'enable_control': True,
                'pcmd_rate_hz': 25.0,
                'deadman_timeout': 0.6,
                'max_vx': 2.0, 'max_vy': 2.0, 'max_vz': 1.0, 'max_yaw_rate': 1.0
            }],
            condition=IfCondition(enable_state),
        ),

        # 팔로워
        Node(
            package='anafi_ai_bridge',
            executable='crazyflie_follower',
            name='crazyflie_follower',
            output='screen',
            parameters=[{
                'image_topic': image_topic,
                'cmd_topic': '/anafi/cmd_vel',
                'ctrl_rate_hz': 20.0,
                'k_yaw': 1.2, 'k_vx': 1.5, 'k_vz': 1.0,
                'max_vx': 1.5, 'max_vz': 1.0, 'max_yaw': 1.0,
                'target_px': 120.0,
                'aruco_id': 0, 'aruco_dict': 'DICT_4X4_50',
                'lost_timeout': 0.5, 'scan_when_lost': True,
            }],
            condition=IfCondition(enable_follower),
        ),
    ])
