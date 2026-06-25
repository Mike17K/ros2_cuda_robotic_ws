FROM nvcr.io/nvidia/isaac/ros:x86_64-humble-nav2_isaac_sim_v3.1

ENV DEBIAN_FRONTEND=noninteractive

# System tools
RUN apt-get update && apt-get install -y \
    git \
    vim \
    nano \
    curl \
    wget \
    build-essential \
    cmake \
    pkg-config \
    python3-pip \
    python3-venv \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-argcomplete \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Create development user
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd --uid ${USER_UID} \
    --gid ${USER_GID} \
    -m \
    -s /bin/bash \
    ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Workspace
RUN mkdir -p /workspaces/isaac_ros_ws && \
    chown -R ${USERNAME}:${USERNAME} /workspaces

USER ${USERNAME}
WORKDIR /workspaces/isaac_ros_ws

# ROS setup for every shell
RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc

# Upgrade pip
RUN pip install --upgrade pip setuptools wheel

CMD ["/bin/bash"]