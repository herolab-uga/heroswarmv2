from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import socket
import numpy as np
def generate_launch_description():
    graph_lapacian = np.array(
        [1,-1,0],
        [-1,2,-1],
        [0,-1,1]
    )
    F = DeclareLaunchArgument("F", default_value="-1")
    g = DeclareLaunchArgument("g", default_value="1")
    L = DeclareLaunchArgument("L", default_value=str(graph_lapacian))

    return LaunchDescription([
        F,
        g,
        L,
        Node(
            package='gravity',
            namespace=socket.gethostname(),
            executable='gravity',
            name='controller',
            output='screen',
            parameters=[
                {
                    "F": LaunchConfiguration("F"),
                    "g": LaunchConfiguration("G"),
                    "L": LaunchConfiguration("L")
                }
            ]
        ),
    ])
