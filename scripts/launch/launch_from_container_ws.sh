#!/bin/bash
# Opens Terminator with the workcell panes - run from the workspace root,
# from a shell INSIDE the container (terminator runs there).
# Each pane gets its command pre-filled on the prompt: press Enter in the
# order of the numbers in the pane titles. Pane plan: scripts/launch/panels.sh.
WS=${PWD}
source "$WS/scripts/launch/panels.sh"
open_panels container "$WS"
