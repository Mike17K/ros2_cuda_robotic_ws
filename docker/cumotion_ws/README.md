# Tech used

[isaac-ros-cli](https://github.com/NVIDIA-ISAAC-ROS/isaac-ros-cli) for handling the setup and running of docker prebuild containers

planner [docs cumotion](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_cumotion/isaac_ros_cumotion/index.html)
[quickstart](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_cumotion/isaac_ros_cumotion_moveit/index.html#quickstart)

# Setup

```bash
sudo bash setup_host.sh
# then in the docker container shell
sudo apt-get update
rosdep update && rosdep install --from-paths src
cd ${ISAAC_ROS_WS} && colcon build --packages-up-to isaac_ros_cumotion_examples
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
