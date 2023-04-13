# isaac_ros_tutorial [![build](https://github.com/rbonghi/isaac_ros_tutorial/actions/workflows/docker-builder.yml/badge.svg)](https://github.com/rbonghi/isaac_ros_tutorial/actions/workflows/docker-builder.yml)

A set of tutorial to define and setup your docker for each package

Basic reference:
* [Isaac ROS GEMs](https://developer.nvidia.com/isaac-ros-gems)
* [Repositories](https://github.com/NVIDIA-ISAAC-ROS)
* [Jetson containers](https://github.com/dusty-nv/jetson-containers)
* Modified from https://github.com/rbonghi/isaac_ros_tutorial

# Tutorial

* [01-argus_camera](01-argus_camera/README.md)
* [02-realsense_camera](02-realsense_camera/README.md)
* [03-zed_camera](03-zed_camera/README.md)
* [04-oakd_camera](03-oakd_camera/README.md) - **In draft**

# Install 
! Make sure that you are running JetPack 4.6.1
* run the script to setup the correct docker environment ```bash scripts/fix_docker.sh```
* build the base image ```bash scripts/build_dockerfile.sh 00-isaac_ros_base```
* build the image of the example you want to run ie. ```bash scripts/build_dockerfile.sh 02-realsense_camera --no-pull-base-image```
* run the docker image ```docker run --rm -it --runtime nvidia --network host rbonghi/isaac-ros-tutorial:realsense-camera```

# View remotely

To install the robot description on your remote machine, follow the readme

* [remote_viewer](remote_viewer/README.md) folder

You will be able to see the an output like

![zedm rviz](remote_viewer/rviz_realsense.png)

# Pre-built Container Images

| Container tag                                 | L4T version |
|-----------------------------------------------|-------------|
| `rbonghi/isaac-ros-tutorial:isaac-ros-base`   | R32.6.1     |
| `rbonghi/isaac-ros-tutorial:argus-camera`     | R32.6.1     |
| `rbonghi/isaac-ros-tutorial:realsense-camera` | R32.6.1     |
| `rbonghi/isaac-ros-tutorial:zed-camera`       | R32.6.1     |

# Notes
This fork contains modification that were necessary to build on my Jetson Nano.
These included:
* Adding Public keys for repositories
* Modifying xacro to be able to build (1)

(1) xacro threw the error: 
```CMake Error at /opt/ros/foxy/install/share/ament_cmake_python/cmake/ament_python_install_package.cmake:34 (message):
ament_python_install_package() called with unused arguments:
 SCRIPTS_DESTINATION;lib/xacro
```
Commenting out the lines that call ```ament_python_install_package``` seems to have solved this for now.
For this reason, this repo replaces the default xacro CMakeLists.txt with a modified file containing this fix
