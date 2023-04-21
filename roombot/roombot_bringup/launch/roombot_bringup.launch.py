from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

import os
import launch
import xacro
import tempfile

def to_urdf(xacro_path, parameters=None):
    """Convert the given xacro file to URDF file.
    * xacro_path -- the path to the xacro file
    * parameters -- to be used when xacro file is parsed.
    """
    urdf_path = tempfile.mktemp(prefix="%s_" % os.path.basename(xacro_path))

    # open and process file
    doc = xacro.process_file(xacro_path, mappings=parameters)
    # open the output file
    out = xacro.open_output(urdf_path)
    out.write(doc.toprettyxml(indent='  '))

    return urdf_path

def generate_launch_description():
    roombot_config = os.path.join(
            get_package_share_directory("roombot_bringup"),
            "config",
            "roombot.yaml"
        )

    roombot_config = DeclareLaunchArgument(
            "roombot_config_file",
            default_value=roombot_config,
            description="Roombot ocnfiguration file"
            )

    create_driver = Node(
        package="create_driver",
        executable="create_driver",
        name="create_driver",
        parameters=[LaunchConfiguration("roombot_config_file")])

    """Launch file which brings up visual odometry node configured for RealSense."""
    realsense_camera_node = Node(
        package='realsense2_camera',
        node_executable='realsense2_camera_node',
        namespace='camera',
        parameters=[{
                'infra_height': 360,
                'infra_width': 640,
                'enable_color': False,
                'enable_depth': False,
                'stereo_module.emitter_enabled': 2, #https://github.com/IntelRealSense/realsense-ros/issues/817
                'infra_fps': 90.0,
                'unite_imu_method': 'linear_interpolation' # copy | linear_interpolation
        }],
        )
    
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        namespace='camera',
        plugin='isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
                    'enable_rectified_pose': False,
                    'denoise_input_images': False,
                    'rectified_images': True,
                    'enable_debug_mode': False,
                    'debug_dump_path': '/tmp/elbrus',
                    'enable_slam_visualization': True,
                    'enable_landmarks_view': True,
                    'enable_observations_view': True,
                    'enable_imu': False,
                    'map_frame': 'map',
                    'odom_frame': 'odom',
                    'base_frame': 'base_footprint',
                    'input_imu_frame': 'camera_imu_optical_frame',
                    'input_left_camera_frame': 'camera_infra1_frame',
                    'input_right_camera_frame': 'camera_infra2_frame'
                    }],
        remappings=[('stereo_camera/left/image', 'infra1/image_rect_raw'),
                    ('stereo_camera/left/camera_info', 'infra1/camera_info'),
                    ('stereo_camera/right/image', 'infra2/image_rect_raw'),
                    ('stereo_camera/right/camera_info', 'infra2/camera_info'),
                    ('visual_slam/imu', 'imu')]
    )

    vslam_launch_container = ComposableNodeContainer(
        name='vslam_launch_container',
        namespace='camera',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            visual_slam_node
        ],
        output='screen'
    )

    roombot_description = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('roombot_description'), 'launch'),
                '/roombot_description.launch.py'])
            )

    xacro_path = os.path.join(get_package_share_directory('roombot_description'), 'urdf', 'roombot_rs.urdf.xacro')
    urdf = to_urdf(xacro_path, {'use_nominal_extrinsics' : 'true', 'add_plug' : 'true'})

    return launch.LaunchDescription([roombot_config, roombot_description, create_driver, vslam_launch_container, realsense_camera_node])
