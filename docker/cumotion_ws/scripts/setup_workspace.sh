#!/bin/bash
sudo apt update && sudo apt install ros-jazzy-moveit-msgs
make rosdeps
make
source install/setup.bash