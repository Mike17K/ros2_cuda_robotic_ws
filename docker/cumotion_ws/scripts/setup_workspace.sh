#!/bin/bash
sudo apt update && sudo apt install ros-jazzy-moveit-msgs
sudo apt-get update && sudo apt-get install -y ros-jazzy-rmw-cyclonedds-cpp
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
make rosdeps
make
source install/setup.bash