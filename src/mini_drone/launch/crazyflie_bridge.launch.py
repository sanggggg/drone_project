from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mini_drone',
            namespace='cf',
            executable='cf_bridge',
            name='cf_bridge',
            output='screen',
            parameters=[{
                'uri': 'radio://0/80/2M/E7E7E7E709',
                'period_ms': 100,
                'publish_rate_hz': 20.0,
                'use_state_estimate': True,
                'hl_goto_duration_s': 5.1, 
            }],
        ),
    ])

