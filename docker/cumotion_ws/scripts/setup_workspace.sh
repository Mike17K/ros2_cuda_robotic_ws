#!/bin/bash
sudo apt update && sudo apt install -y ros-jazzy-moveit-msgs ros-jazzy-rmw-cyclonedds-cpp
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=40" >> ~/.bashrc
source ~/.bashrc
make rosdeps
make
source install/setup.bash