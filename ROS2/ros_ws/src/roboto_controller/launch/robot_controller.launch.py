from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import socket

def generate_launch_description():

    #
    # Boolean toggles (defaults same as your project)
    #
    all_sensors_arg = DeclareLaunchArgument("all_sensors", default_value="True")
    mic_arg         = DeclareLaunchArgument("mic", default_value="True")
    proximity_arg   = DeclareLaunchArgument("proximity", default_value="False")
    light_arg       = DeclareLaunchArgument("light", default_value="False")
    imu_arg         = DeclareLaunchArgument("imu", default_value="True")
    environment_arg = DeclareLaunchArgument("environment", default_value="False")
    global_pos_arg  = DeclareLaunchArgument("global_pos", default_value="False")

    #
    # SHT31D toggles + publish rate
    #
    sht31d_arg = DeclareLaunchArgument(
        "sht31d",
        default_value="True"
    )

    sht31d_pub_rate_arg = DeclareLaunchArgument(
        "sht31d_publish_rate",
        default_value="5.0"        # 5 Hz default
    )

    #
    # LIS3MDL parameters (NO AUTODETECT)
    #
    lis3mdl_range_arg = DeclareLaunchArgument(
        "lis3mdl_range_gauss",
        default_value="4"   # 4 gauss default
    )

    lis3mdl_rate_arg = DeclareLaunchArgument(
        "lis3mdl_data_rate",
        default_value="155" # 155 Hz default
    )

    lis3mdl_pub_arg = DeclareLaunchArgument(
        "lis3mdl_publish_rate",
        default_value="50.0" # 50 Hz
    )

    return LaunchDescription([
        all_sensors_arg,
        mic_arg,
        proximity_arg,
        light_arg,
        imu_arg,
        environment_arg,
        global_pos_arg,

        # SHT31D
        sht31d_arg,
        sht31d_pub_rate_arg,

        # LIS3MDL
        lis3mdl_range_arg,
        lis3mdl_rate_arg,
        lis3mdl_pub_arg,

        #
        # Robot Controller
        #
        Node(
            package="roboto_controller",
            namespace=socket.gethostname(),
            executable="robot_controller",
            name="controller",
            output="screen"
        ),

        #
        # Sensor Manager
        #
        Node(
            package="roboto_controller",
            namespace=socket.gethostname(),
            executable="sensor_manager",
            name="sensor_manager",
            output="screen",
            parameters=[
                {
                    "all_sensors": LaunchConfiguration("all_sensors"),
                    "mic": LaunchConfiguration("mic"),
                    "proximity": LaunchConfiguration("proximity"),
                    "light": LaunchConfiguration("light"),
                    "imu": LaunchConfiguration("imu"),
                    "environment": LaunchConfiguration("environment"),
                    "global_pos": LaunchConfiguration("global_pos"),

                    #
                    # SHT31D Parameters
                    #
                    "sht31d": LaunchConfiguration("sht31d"),
                    "sht31d_publish_rate": LaunchConfiguration("sht31d_publish_rate"),

                    #
                    # LIS3MDL Parameters (NO AUTODETECT)
                    #
                    "lis3mdl_range_gauss": LaunchConfiguration("lis3mdl_range_gauss"),
                    "lis3mdl_data_rate": LaunchConfiguration("lis3mdl_data_rate"),
                    "lis3mdl_publish_rate": LaunchConfiguration("lis3mdl_publish_rate"),
                }
            ]
        )
    ])
