#!/bin/bash
# First-boot setup inside the workspace container (also run by Dockerfile.cumotion_ws).
# Safe to re-run: apt skips installed packages and ~/.bashrc lines are added once.
# Run from the workspace root.
set -e

ROS_DOMAIN_ID=40
BASHRC=~/.bashrc

SUDO=""
[ "$(id -u)" != "0" ] && SUDO="sudo"

$SUDO apt-get update
$SUDO apt-get install -y \
    ros-jazzy-moveit-msgs \
    ros-jazzy-rmw-cyclonedds-cpp \
    terminator

# Append a line to ~/.bashrc only if it is not there yet.
add_to_bashrc() {
    grep -qxF "$1" "$BASHRC" 2>/dev/null || echo "$1" >> "$BASHRC"
}
add_to_bashrc "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
add_to_bashrc "export ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

# Build the workspace (the install/ overlay is sourced by new shells / pane.sh)
make rosdeps
make
