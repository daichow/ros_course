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

# Get gazebo binaries
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list \
 && wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - \
 && apt-get update \
 && apt-get install -y \
    gazebo11 \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-fake-localization \
    ros-noetic-joy \
 && apt-get clean

# git clone and rosdep
RUN git -C src clone \
      --branch noetic \
      https://bitbucket.org/theconstructcore/bb8 
RUN apt-get update
RUN rosdep install --from-paths src --ignore-src -r -y 
RUN rm -rf /var/lib/apt/lists/*


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
