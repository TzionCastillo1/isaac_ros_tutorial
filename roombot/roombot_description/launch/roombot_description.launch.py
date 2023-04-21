import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    xacro_file_name = 'roombot_rs.urdf.xacro'
    xacro_file = os.path.join(get_package_share_directory('roombot_description'), 'urdf',
            xacro_file_name)
    #Convert xacro to xml
    robot_desc = xacro.process_file(xacro_file).toxml()

    #configure the node
    return LaunchDescription([
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': robot_desc}]
                ),
    ])

