# Tech used

[isaac-ros-cli](https://github.com/NVIDIA-ISAAC-ROS/isaac-ros-cli) for handling the setup and running of docker prebuild containers

planner [docs cumotion](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_cumotion/isaac_ros_cumotion/index.html)
[quickstart](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_cumotion/isaac_ros_cumotion_moveit/index.html#quickstart)

# Scripts

- `scripts/setup_host.sh` : should be runed in the host for setup the proper packages to handle docker setup
- `scripts/shell.sh` : should be runed in the host, creates container and attaches the shell inside it (user admin)
- `scripts/setup_workspace.sh` : should be runed in the container after the `shell.sh` and setups and builds the workspace

# Setup

on the host

```bash
sudo bash scripts/setup_host.sh
bash scripts/shell.sh
```

the shell gets you in the isaac ros container

```bash
bash scripts/setup_workspace.sh
```

# Run

```bash
bash shell.sh
source install/setup.bash

```

# Rosbridge

```
source /opt/ros/jazzy/setup.bash
sudo -E npm install -g ros2-web-bridge --unsafe-perm
```
