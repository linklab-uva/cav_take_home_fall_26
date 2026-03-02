FROM osrf/ros:humble-desktop

ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=1000

# Create non-root user with sudo access
RUN groupadd --gid $USER_GID $USERNAME || echo "Group already exists" \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME || echo "User already exists" \
    && apt-get update \
    && apt-get install -y sudo \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# Install essential development tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    wget \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    vim \
    nano \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 packages needed for the take-home
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-foxglove-bridge \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-rviz-common \
    && rm -rf /var/lib/apt/lists/*

# Setup bashrc for the user
RUN echo 'source /opt/ros/humble/setup.bash' >> /home/$USERNAME/.bashrc \
    && chown $USERNAME:$USERNAME /home/$USERNAME/.bashrc

ENV ROS_DOMAIN_ID=42

WORKDIR /home/$USERNAME/cav_take_home_fall_26

# Switch to the user
USER $USERNAME
ENV SHELL=/bin/bash
CMD ["/bin/bash"]
