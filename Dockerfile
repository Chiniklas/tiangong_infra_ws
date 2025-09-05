# For CUDA + OpenGL GUI support
FROM osrf/ros:noetic-desktop-full
# OR for headless only: FROM nvidia/cuda:11.8.0-devel-ubuntu20.04

ENV DEBIAN_FRONTEND=noninteractive

# ROS Noetic setup
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1-latest.list' \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add -
RUN apt-get update && apt-get install -y \
    ros-noetic-desktop-full \
    python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool \
    build-essential git cmake python3-pip \
    ros-noetic-moveit ros-noetic-ros-control ros-noetic-ros-controllers \
    ros-noetic-robot-state-publisher ros-noetic-joint-state-publisher-gui \
    ros-noetic-diagnostics ros-noetic-rqt ros-noetic-rqt-common-plugins \
    mesa-utils libgl1 libglfw3 libxkbcommon0 libxi6 libxinerama1 libxcursor1 libxxf86vm1 libxrandr2 \
 && rm -rf /var/lib/apt/lists/*

# Python deps (incl. MuJoCo)
RUN pip3 install --no-cache-dir mujoco==3.1.6 numpy scipy matplotlib pandas pyyaml transforms3d ipython ipdb

# at the end of your Dockerfile
RUN echo 'export ROS_HOSTNAME=localhost' >> /root/.bashrc && \
    echo 'export ROS_MASTER_URI=http://localhost:11311' >> /root/.bashrc

