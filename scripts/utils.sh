#!/bin/bash

# Check for variables needed
variable_names=("WS" "LAYOUT_NAME" "TERMINATOR_CONFIG" "DEFAULT_DELAY" "DEFAULT_LONG_DELAY")
for var in "${variable_names[@]}"; do
    if [ -z "${!var}" ]; then
        echo "Error: Variable '$var' is not set. Please set it before running the script."
        exit 1
    fi
done

# Ενεργοποίηση των aliases μέσα στο script
shopt -s expand_aliases

# Αρχικό Setup Περιβάλλοντος
source /opt/ros/jazzy/setup.bash
[ -f ".venv/bin/activate" ] && source .venv/bin/activate
[ -f "install/setup.bash" ] && source install/setup.bash

# Aliases για πλοήγηση και λειτουργίες
alias move_up="xdotool key Alt+Up && sleep $DEFAULT_DELAY"
alias move_down="xdotool key Alt+Down && sleep $DEFAULT_DELAY"
alias move_left="xdotool key Alt+Left && sleep $DEFAULT_DELAY"
alias move_right="xdotool key Alt+Right && sleep $DEFAULT_DELAY"
alias broadcast_on="xdotool key Super+g && sleep $DEFAULT_LONG_DELAY && xdotool key shift+ctrl+a && sleep $DEFAULT_DELAY"
alias broadcast_off="xdotool key Super+g && sleep $DEFAULT_LONG_DELAY && xdotool key shift+ctrl+h && $DEFAULT_DELAY"
alias enter="xdotool key Return && sleep $DEFAULT_DELAY"
alias split_vertical="xdotool key ctrl+shift+e && sleep $DEFAULT_DELAY"
alias split_horizontal="xdotool key ctrl+shift+o && sleep $DEFAULT_DELAY"

# Συνάρτηση για επικόλληση και εκτέλεση εντολής
paste_cmd() {
    local text="$1"
    echo -n "$text" | xclip -selection clipboard
    sleep $DEFAULT_DELAY
    xdotool key ctrl+shift+v
    sleep $DEFAULT_DELAY
}

open_terminator() {
    echo "Launching terminator with layout: $LAYOUT_NAME"
    terminator -u -g $TERMINATOR_CONFIG -l $LAYOUT_NAME &
    sleep 0.5

    # 2. Εστίαση στο παράθυρο του Terminator
    # MAX_RETRIES=10
    # WID=""
    # while [ -z "$WID" ] && [ $MAX_RETRIES -gt 0 ]; do
    #     WID=$(xdotool search --onlyvisible --class "terminator" | tail -1)
    #     [ -z "$WID" ] && sleep 1
    #     ((MAX_RETRIES--))
    # done

    # if [ -z "$WID" ]; then
    #     echo "Error: Terminator window not found."
    #     exit 1
    # fi
    
    # xdotool windowactivate $WID
}
