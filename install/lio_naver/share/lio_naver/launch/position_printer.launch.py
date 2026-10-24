from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lio_naver',
            executable='position_printer',
            name='position_printer',
            output='screen',
            parameters=[{
                'odom_topic': '/aft_mapped_to_init',
                'print_rate': 10.0,
                'show_orientation': True,

            }]
        )
    ])