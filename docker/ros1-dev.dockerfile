FROM osrf/ros:noetic-desktop-full

# use bash instead of sh
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# update system image and install dependencies
RUN apt-get update && apt upgrade -y
RUN apt-get install -y \
    software-properties-common \
    wget \
    git \
    python3-dev \
    python3-pip \
    python3-rospkg \
    python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

# create workspace
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws

# compile packages
RUN catkin config --extend /opt/ros/$ROS_DISTRO \
    && catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && catkin build

# add sourcing catkine_ws to entrypoint
RUN sed --in-place --expression \
    '$isource "/catkin_ws/devel/setup.bash"' \
    /ros_entrypoint.sh

# add packages to path
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc