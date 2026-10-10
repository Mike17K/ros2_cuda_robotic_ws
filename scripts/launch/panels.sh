#!/bin/bash
# Pane plan for launch_from_host_ws.sh / launch_from_container_ws.sh, plus the
# Terminator config generator. Sourced by those scripts and by pane.sh.
#
# Every pane runs scripts/launch/pane.sh (Terminator's per-terminal
# `command`), which prepares the ROS environment inside the container and
# leaves the pane's command pre-filled on the prompt: check it, press Enter.
# No keystroke automation (xdotool) is involved, so a command can no longer
# land in the wrong pane.
#
# Suggested start order = the number in each pane title. The planner, nvblox
# and RViz need the robot up (joint states, TF).

ROS_DOMAIN_ID=40  # matches the container root .bashrc (setup_workspace.sh)
ROS_DISTRO="jazzy"
CONTAINER_WS="/workspaces/isaac_ros-dev"

# "title|command" - left column top to bottom, then the right column.
# An empty command gives a plain prepared shell.
PANES_LEFT=(
    "1 Workcell (Gazebo)|ros2 launch workcell_bringup workcell.launch.py sim_gazebo:=true use_fake_hardware:=false"
    "3 nvblox|ros2 launch vision nvblox.launch.py"
)
PANES_RIGHT=(
    "2 cuMotion planner|ros2 launch planning_bringup cumotion.launch.py"
    "4 RViz|ros2 launch workcell_bringup rviz.launch.py rviz_namespace:=robot_1"
)
PANES=("${PANES_LEFT[@]}" "${PANES_RIGHT[@]}")

# Environment every pane gets inside the container. The venv is optional (a
# fresh container has none).
pane_env() {
    cd "$CONTAINER_WS" || return 1
    source /opt/ros/$ROS_DISTRO/setup.bash
    [ -f "$CONTAINER_WS/install/setup.bash" ] && source "$CONTAINER_WS/install/setup.bash"
    [ -f "$CONTAINER_WS/.venv/bin/activate" ] && source "$CONTAINER_WS/.venv/bin/activate"
    export ISAAC_ROS_WS=$CONTAINER_WS
    export PYTHONPATH=$PYTHONPATH:$CONTAINER_WS/external
    export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    [ -f "$CONTAINER_WS/cyclonedds.xml" ] && export CYCLONEDDS_URI=file://$CONTAINER_WS/cyclonedds.xml
}

_terminal() {  # _terminal <pane index> <parent> <order>
    local title=${PANES[$1]%%|*}
    printf '    [[[terminal%s]]]\n      type = Terminal\n      parent = %s\n      order = %s\n      title = %s\n      command = %s %s\n' \
        "$1" "$2" "$3" "$title" "$PANE_LAUNCHER" "$1"
}

# One column of `count` equal-height rows: a chain of VPaneds, each splitting
# off the top row with ratio 1/(rows left) - so all rows get the same height.
_column() {  # _column <parent> <order> <name> <first pane index> <count>
    local parent=$1 order=$2 name=$3 first=$4 count=$5 i
    local up=$parent ord=$order
    for ((i = 0; i < count - 1; i++)); do
        printf '    [[[%s_%s]]]\n      type = VPaned\n      parent = %s\n      order = %s\n      ratio = %s\n' \
            "$name" "$i" "$up" "$ord" "$(awk -v n=$((count - i)) 'BEGIN{printf "%.4f", 1.0 / n}')"
        _terminal "$((first + i))" "${name}_$i" 0
        up="${name}_$i"
        ord=1
    done
    _terminal "$((first + count - 1))" "$up" "$ord"
}

# write_terminator_config <file> <layout name> - PANE_LAUNCHER must be the
# command that runs pane.sh for a pane index (host or container side).
write_terminator_config() {
    local file=$1 layout=$2
    {
        cat <<'EOF'
[global_config]
  suppress_multiple_term_dialog = True
[keybindings]
  broadcast_off = <Primary><Shift>h
  broadcast_all = <Primary><Shift>a
[profiles]
  [[default]]
    scrollback_lines = 5000
[layouts]
EOF
        printf '  [[%s]]\n    [[[window0]]]\n      type = Window\n      parent = ""\n' "$layout"
        printf '    [[[split_horiz]]]\n      type = HPaned\n      parent = window0\n      ratio = 0.5\n'
        _column split_horiz 0 left 0 ${#PANES_LEFT[@]}
        _column split_horiz 1 right ${#PANES_LEFT[@]} ${#PANES_RIGHT[@]}
        echo "[plugins]"
    } > "$file"
}

# open_panels <host|container> <workspace path on this side> - write the
# config and start Terminator.
open_panels() {
    local mode=$1 ws=$2 config
    PANE_LAUNCHER="bash $ws/scripts/launch/pane.sh $mode"
    config=$(mktemp /tmp/diffusion_terminator.XXXXXX)
    write_terminator_config "$config" GazeboLayout
    echo "Launching Terminator with $config"
    terminator -u -g "$config" -l GazeboLayout &
}
