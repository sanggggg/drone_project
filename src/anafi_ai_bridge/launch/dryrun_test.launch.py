from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    image_topic = LaunchConfiguration('image_topic')

    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/anafi_ai/image_raw'),

        Node(
            package='gscam', executable='gscam_node', name='gscam_node', output='screen',
            parameters=[{
                'gscam_config':
                    'videotestsrc is-live=true pattern=ball ! videoconvert ! '
                    'video/x-raw,format=BGR ! appsink name=appsink',
                'camera_name': 'anafi_ai',
                'frame_id': 'anafi_ai_camera',
            }],
            remappings=[('/image_raw', image_topic)],
        ),

        Node(
            package='anafi_ai_bridge', executable='crazyflie_follower',
            name='crazyflie_follower', output='screen',
            parameters=[{
                'image_topic': image_topic,
                'cmd_topic': '/anafi/cmd_vel',
                'scan_when_lost': True,
            }],
        ),
    ])
