from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    all_sensors_arg = DeclareLaunchArgument("all_sensors", default_value="False")
    mic_arg = DeclareLaunchArgument("mic", default_value="True")
    proximity_arg = DeclareLaunchArgument("proximity", default_value="False")
    light_arg = DeclareLaunchArgument("light", default_value="False")
    imu_arg = DeclareLaunchArgument("imu", default_value="True")
    environment_arg = DeclareLaunchArgument("environment", default_value="False")
    global_pos_arg = DeclareLaunchArgument("global_pos", default_value="False")
    

    return LaunchDescription([
        Node(
            package='robot_controller',
            namespace="$(env HOSTNAME)",
            executable='robot_controller',
            name='$(env HOSTNAME)_controller',
            output='screen',
            parameters=[
                {
                    "all_sensors": LaunchConfiguration("all_sensors"),
                    "mic": LaunchConfiguration("mic"),
                    "proximity": LaunchConfiguration("proximity"),
                    "light": LaunchConfiguration("light"),
                    "imu": LaunchConfiguration("imu"),
                    "environment": LaunchConfiguration("environment"),
                    "global_pos": LaunchConfiguration("global_pos")
                }
            ]
        ),
    ])
