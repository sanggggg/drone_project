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

    return LaunchDescription([
        # ---- Arguments
        DeclareLaunchArgument('ip', default_value='192.168.53.1'),
        DeclareLaunchArgument('camera_name', default_value='anafi_ai'),
        DeclareLaunchArgument('frame_id', default_value='anafi_ai_camera'),
        DeclareLaunchArgument('image_topic', default_value='/anafi_ai/image_raw'),
        DeclareLaunchArgument('enable_state', default_value='true'),
        DeclareLaunchArgument(
            'gscam_config',
            # Skycontroller4 경유(192.168.53.1) 기본 파이프라인
            default_value=(
                'rtspsrc location=rtsp://192.168.53.1/live protocols=tcp latency=100 ! '
                'rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! '
                'video/x-raw,format=BGR ! appsink name=appsink sync=false drop=true max-buffers=1'
            ),
        ),

        # ---- Camera: gscam (apt 패키지의 실행파일 이름은 gscam_node)
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

        # ---- State bridge (Olympe): 필요 시 끄려면 enable_state:=false
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
            }],
            condition=IfCondition(enable_state),
        ),
    ])
