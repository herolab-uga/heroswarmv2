#!/usr/bin/env python3

import os
import socket

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                name=socket.gethostname(),
                package='robot_controller',
                executable='controller',
                namespace=socket.gethostname(),

            )
        ]
    )