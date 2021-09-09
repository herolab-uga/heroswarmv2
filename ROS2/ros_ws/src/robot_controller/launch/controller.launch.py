#!/usr/bin/env python3

import os
import socket
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return(
        [
            Node(
                name=socket.gethostname(),
                package='robot_controller',
                executable='controller',
                namespace=socket.gethostname(),
            )
        ]
    )