import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Set the path to find necessary files and folders
    pkg_share = FindPackageShare(package='roombot+nav2_bringup').find('roombot_nav2_bringup')
    default_params_path = os.path.join(pkg_share, 'params', 'nav2_params.yaml')

    # Launch config variables
    params_file = LaunchConfiguration('params_file')

    declare_params_file = DeclareLaunchArgument(
            name='params_file',
            default_value=default_params_path,
            description='Full path to the ROS2 parameters file to use for launched nodes')

    start_cmd_vel_mux = Node(
            package="cmd_vel_mux",
            executable="cmd_vel_mux_node",
            name="cmd_vel_mux_node",
            output="screen",
            parameters=[params_file])

    ld = LaunchDescription()

    ld.add_action(start_cmd_vel_mux)

    return ld
