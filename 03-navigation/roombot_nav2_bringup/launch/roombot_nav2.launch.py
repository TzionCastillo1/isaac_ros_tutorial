# this files deals only with the nav2 side of the roombot

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Set the path to different files and folders
    pkg_share = FindPackageShare(package='roombot_nav2_bringup').find('roombot_nav2_bringup')
    nav2_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup')
    nav2_launch_dir = os.path.join(nav2_dir, 'launch')
    nav2_params_path = os.path.join(pkg_share, 'params', 'nav2_params.yaml')
    nav2_bt_path = FindPackageShare(package='nav2_bt_navigator').find('nav2_bt_navigator')
    behavior_tree_xml_path = os.path.join(nav2_bt_path, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')
    tmux_config_path = os.path.join(pkg_share, 'params', 'tmux_config.yaml')

    # Launch config variables
    map_yaml_file = LaunchConfiguration('map')
    namespace = LaunchConfiguration('namesapce')
    params_file = LaunchConfiguration('params_file')
    use_namespace = LaunchConfiguration('use_namespace')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    autostart = LaunchConfiguration('autostart')

    declare_autostart_cmd = DeclareLaunchArgument(
            name='autostart',
            default_value='true',
            description='Automatically startup the nav2 stack')

    declare_namespace_cmd = DeclareLaunchArgument(
            name='namespace',
            default_value='',
            description='Top-level namesapce')

    declare_use_namespace_cmd = DeclareLaunchArgument(
            name='use_namespace',
            default_value='False',
            description='Whether to apply a namespace to the navigation stack')

    declare_map_yaml_cmd = DeclareLaunchArgument(
            name='map',
            default_value='',
            description='Full path to map file to load')

    declare_params_file = DeclareLaunchArgument(
            name='params_file',
            default_value=nav2_params_path,
            description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_bt_xml_cmd = DeclareLaunchArgument(
            name='default_bt_xml_filename',
            default_value=behavior_tree_xml_path,
            description='Full path to the behavior tree xml file to use')

    start_ros2_navigation_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
            launch_arguments = {'namespace': namespace,
                                'use_namespace': use_namespace,
                                'slam': False,
                               # 'map': map_yaml_file,
                                'use_sim_time': False,
                                'params_file': params_file,
                                'autostart': autostart}.items())

    start_cmd_vel_mux = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share,'launch', 'roombot_cmd_vel_mux.launch.py')),
            launch_arguments = {'params_file': params_file}.items())

    #start_cmd_vel_mux = Node(
    #        package="cmd_vel_mux",
    #        executable="cmd_vel_mux_node",
    #        name="cmd_vel_mux_node",
    #        output="screen",
    #        parameters=[params_file])

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file)
    ld.add_action(declare_bt_xml_cmd)

    # Add any actions
    ld.add_action(start_ros2_navigation_cmd)
    ld.add_action(start_cmd_vel_mux)

    return ld
