from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_server',
            executable='camera_server',
            name='camera_server',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='camera_server',
            executable='openCV_publisher',
            name='openCV_publisher',
            output='screen',
            emulate_tty=True
        )
    ])