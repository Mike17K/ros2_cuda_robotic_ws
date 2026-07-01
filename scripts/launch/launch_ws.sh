#!/bin/bash

ROS_DOMAIN_ID=55
ROS_DISTRO="jazzy"
WS=${PWD}
DEFAULT_DELAY=0.05
DEFAULT_LONG_DELAY=0.2
# Η εντολή που προετοιμάζει κάθε νέο terminal panel
GLOBAL_CMD="cd $WS && source /opt/ros/$ROS_DISTRO/setup.bash && source $WS/install/setup.bash && source $WS/.venv/bin/activate && export PYTHONPATH=\$PYTHONPATH:$WS/external && export ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
LAYOUT_NAME="GazeboLayout"
TERMINATOR_CONFIG="$WS/scripts/config/terminator_config"

source $WS/scripts/utils.sh

open_terminator


# # 3. Προετοιμασία: Σιγουρεύουμε ότι είμαστε στο πάνω panel
move_up
move_left

# configuration broadcasting
echo "Enabling broadcasting for all panels..."
broadcast_on
paste_cmd "$GLOBAL_CMD && clear" 
enter
broadcast_off

# --- PANEL 1 (Πάνω): Camera Input Node ---
echo "Configuring Panel 1..."
paste_cmd 'ros2 launch workcell_bringup workcell.launch.py sim_gazebo:=true use_fake_hardware:=false'
# enter


move_right
paste_cmd "ros2 launch vision nvblox.launch.py robots:=robot_1,robot_2"
move_right
# enter

move_down
paste_cmd "ros2 launch workcell_bringup rviz.launch.py rviz_namespace:=robot_1"

move_left
paste_cmd "ros2 run tf2_ros static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 1.0 map group_a/odom"