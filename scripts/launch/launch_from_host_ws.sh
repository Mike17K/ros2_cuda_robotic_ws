#!/bin/bash
# Opens Terminator with the workcell panes - run from the workspace root,
# from a HOST terminal: every pane docker-execs into the running workspace container.
# Each pane gets its command pre-filled on the prompt: press Enter in the
# order of the numbers in the pane titles. Pane plan: scripts/launch/panels.sh.
WS=${PWD}
source "$WS/scripts/launch/panels.sh"
open_panels host "$WS"
