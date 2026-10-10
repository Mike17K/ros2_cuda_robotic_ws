#!/bin/bash
# One Terminator pane (see panels.sh): pane.sh <host|container> <pane index>
#
#   host       re-runs itself inside the workspace container (docker exec)
#   container  prepares the ROS environment, puts the pane's command on the
#              prompt pre-filled (edit it, Enter runs it, Ctrl+C skips it) and
#              leaves an interactive shell behind (the command is in history).
MODE=$1
INDEX=$2
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$HERE/panels.sh"

if [ "$MODE" = "host" ]; then
    CONTAINER=${WORKSPACE_CONTAINER:-}
    if [ -z "$CONTAINER" ]; then
        for name in nvidia_workspace_container isaac_ros_dev_container; do
            if docker ps --format '{{.Names}}' | grep -qx "$name"; then CONTAINER=$name; break; fi
        done
    fi
    if [ -z "$CONTAINER" ]; then
        echo "No workspace container running - start it with: bash scripts/shell.sh"
        echo "(or set WORKSPACE_CONTAINER=<name>), then reopen the panels."
        exec bash
    fi
    exec docker exec -it "$CONTAINER" bash "$CONTAINER_WS/scripts/launch/pane.sh" container "$INDEX"
fi

pane_env
ENTRY=${PANES[$INDEX]}
TITLE=${ENTRY%%|*}
CMD=${ENTRY#*|}

# Leave a prepared interactive shell whatever happens (Ctrl+C, command exit).
trap 'echo; exec bash -i' INT
printf '\033]0;%s\007' "$TITLE"   # keep the pane title
echo "== $TITLE =="
if [ -n "$CMD" ]; then
    echo "Enter runs it (edit first if needed), Ctrl+C skips it."
    PROMPT_CHAR='$'; [ "$(id -u)" = 0 ] && PROMPT_CHAR='#'
    read -r -e -p "$(whoami)@$(hostname):$PWD$PROMPT_CHAR " -i "$CMD" LINE
    if [ -n "$LINE" ]; then
        echo "$LINE" >> ~/.bash_history
        eval "$LINE"
    fi
fi
exec bash -i
