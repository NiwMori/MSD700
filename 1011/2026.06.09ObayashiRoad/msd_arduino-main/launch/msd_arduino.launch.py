from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'port', default_value='/dev/arduino-mega-uart',
        description='Serial device for the Arduino Mega LINK UART (Serial3 via FTDI)'
    )
    baud_arg = DeclareLaunchArgument(
        'baud', default_value='115200',
        description='Serial baud rate'
    )
    hb_arg = DeclareLaunchArgument(
        'heartbeat_period_ms', default_value='50',
        description='Host heartbeat period in milliseconds'
    )
    frame_arg = DeclareLaunchArgument(
        'frame_id_imu', default_value='imu_link',
        description='frame_id stamped on published IMU messages'
    )

    node = Node(
        package='msd_arduino',
        executable='msd_arduino_node',
        name='msd_arduino_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baud': LaunchConfiguration('baud'),
            'heartbeat_period_ms': LaunchConfiguration('heartbeat_period_ms'),
            'frame_id_imu': LaunchConfiguration('frame_id_imu'),
        }],
    )

    return LaunchDescription([port_arg, baud_arg, hb_arg, frame_arg, node])
