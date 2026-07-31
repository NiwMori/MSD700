from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='msd_arduino',
            executable='msd_arduino_dummy_node',
            name='msd_arduino_dummy_node',
            output='screen',
            parameters=[{
                'publish_rate_hz': 20.0,
                'control_mode': 0,
                'relay_on': True,
                'hw_estop': False,
            }],
        ),
    ])
