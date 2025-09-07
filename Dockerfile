# Dockerfile (Kinetic)
FROM osrf/ros:kinetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# Core tooling (note: Kinetic is Python2-first, but we can add Python3 alongside if needed)
RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo git curl wget vim nano bash-completion build-essential cmake \
    python-pip python-setuptools \
    python-catkin-tools \
    # GUI / OpenGL for RViz
    mesa-utils libgl1 libglfw3 \
    libxrandr2 libxinerama1 libxcursor1 libxi6 libxxf86vm1 libxkbcommon0 \
    # MoveIt + control (Kinetic)
    ros-kinetic-moveit \
    ros-kinetic-ros-control ros-kinetic-ros-controllers \
    ros-kinetic-robot-state-publisher ros-kinetic-joint-state-publisher-gui \
    ros-kinetic-diagnostic-updater ros-kinetic-diagnostics \
    ros-kinetic-serial \
 && rm -rf /var/lib/apt/lists/*

# (Optional) Add Python3 if you really need it for tools (NOT for rospy in Kinetic)
# RUN apt-get update && apt-get install -y python3 python3-pip && rm -rf /var/lib/apt/lists/*

# Quality of life: source ROS
RUN echo "source /opt/ros/kinetic/setup.bash" >> /root/.bashrc
WORKDIR /tiangong_infra_ws

CMD ["/bin/bash"]


