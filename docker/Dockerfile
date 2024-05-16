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

# Install additional Python modules
RUN pip3 install matplotlib transforms3d

# Use Cyclone DDS as middleware
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN touch /home/.bashrc
RUN echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> /home/.bashrc
RUN echo 'source /home/abu2024_ws/install/setup.bash' >> /home/.bashrc

# Build the base Colcon workspace, installing dependencies first.
WORKDIR /home/
RUN git clone https://github.com/Entity014/abu2024_ws.git

RUN cd /home/abu2024_ws \ 
    && rosdep update \
    && apt-get update -y \
    && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
    && colcon build --symlink-install



# Download Groot2 AppImage and place it in the home folder.
WORKDIR /root/
RUN curl -o Groot2.AppImage https://s3.us-west-1.amazonaws.com/download.behaviortree.dev/groot2_linux_installer/Groot2-v1.0.1-x86_64.AppImage \
    && chmod a+x Groot2.AppImage

# Remove display warnings
RUN mkdir /tmp/runtime-root
ENV XDG_RUNTIME_DIR "/tmp/runtime-root"
RUN chmod -R 0700 /tmp/runtime-root
ENV NO_AT_BRIDGE 1

# # Set up the entrypoint
# # WORKDIR /home/workspace_ws
# COPY ./docker/entrypoint.sh /
# RUN chmod +x /entrypoint.sh
# ENTRYPOINT [ "/entrypoint.sh" ]

WORKDIR /home
