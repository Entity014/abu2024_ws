# ROS distribution to use
ARG ROS_DISTRO=humble

# Use the official ROS2 Docker image as the base image
FROM osrf/ros:${ROS_DISTRO}-desktop as base
ENV ROS_DISTRO=${ROS_DISTRO}
ENV HOME=/home
SHELL ["/bin/bash", "-c"]

# Install basic apt packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    git libcanberra-gtk-module libcanberra-gtk3-module fuse3 libfuse2 libqt5svg5-dev \
    python3-pip python3-opencv python3-tk python3-pyqt5.qtwebengine nano gedit
# Install Nav2 packages
RUN apt install -y ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-bringup 

# Install GZ Simulation packages
RUN apt install -y ros-${ROS_DISTRO}-turtlebot3-gazebo

# Install Realsense SDK
RUN mkdir -p /etc/apt/keyrings \
    && curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp \
    | tee /etc/apt/keyrings/librealsense.pgp > /dev/null \
    && echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" \
    | tee /etc/apt/sources.list.d/librealsense.list 
RUN apt-get update \
    && apt-get install -y librealsense2-dkms librealsense2-utils

# Install additional Python modules
RUN pip3 install matplotlib transforms3d ultralytics

# Use Cyclone DDS as middleware
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN touch /home/.bashrc
RUN echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> /home/.bashrc \
    && echo 'source /home/robot_ws/install/setup.bash' >> /home/.bashrc

RUN apt-get install -y ros-${ROS_DISTRO}-xacro ros-${ROS_DISTRO}-laser-filters \
    ros-${ROS_DISTRO}-robot-localization ros-${ROS_DISTRO}-controller-manager \ 
    ros-${ROS_DISTRO}-ros2-control ros-${ROS_DISTRO}-ros2-controllers ros-${ROS_DISTRO}-gazebo-ros2-control \
    ros-${ROS_DISTRO}-joint-state-publisher
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-realsense2-* \
    ros-${ROS_DISTRO}-tf-transformations  ros-${ROS_DISTRO}-rtabmap-ros ros-${ROS_DISTRO}-imu-tools
RUN apt-get upgrade -y

# Create abu workspace
# RUN mkdir -p home/abu2024_ws/src

# Build the base Colcon workspace, installing dependencies first.
# WORKDIR /home/abu2024_ws
# RUN rosdep update \
#     && apt-get update -y \
#     && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
#     && colcon build --symlink-install



# Download Groot2 AppImage and place it in the home folder.
WORKDIR /root/
RUN curl -o Groot2.AppImage https://s3.us-west-1.amazonaws.com/download.behaviortree.dev/groot2_linux_installer/Groot2-v1.0.1-x86_64.AppImage \
    && chmod a+x Groot2.AppImage

# Remove display warnings
RUN mkdir /tmp/runtime-root
ENV XDG_RUNTIME_DIR "/tmp/runtime-root"
RUN chmod -R 0700 /tmp/runtime-root
ENV NO_AT_BRIDGE 1

# Set up the entrypointe
# COPY ./docker/entrypoint.sh /
# RUN chmod +x /entrypoint.sh
# ENTRYPOINT [ "/entrypoint.sh" ]

WORKDIR /home/robot_ws
