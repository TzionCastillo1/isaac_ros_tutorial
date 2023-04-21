
# Start from ARM64 ROS2 image
ARG BASE_IMAGE="ros:foxy"
FROM ${BASE_IMAGE}

########################## INSTALL NAV2 ############################
RUN apt-get update && \
    apt-get install -y ros-foxy-navigation2 ros-foxy-nav2-bringup && \
    rm -rf /var/lib/apt/lists/*

################### BUILD AND INSTALL ROS PACKAGES #################
ENV ROS_WS /opt/ros_ws
ARG ROSINSTALL=03-navigation.rosinstall

COPY ${ROSINSTALL} ${ROSINSTALL}

RUN mkdir -p ${ROS_WS}/src && \
    vcs import ${ROS_WS}/src < ${ROSINSTALL}

WORKDIR $ROS_WS

run apt-get update && \
    . /opt/ros/foxy/setup.sh && \
    rosdep install -y --ignore-src --from-paths src --rosdistro foxy && \
    colcon build --symlink-install
