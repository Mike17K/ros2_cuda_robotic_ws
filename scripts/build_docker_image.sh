#!/bin/bash

# Αποθήκευση του αρχικού φακέλου
ORIG_DIR=$(pwd)

function abort {
    echo "Σφάλμα: $1" >&2
    cd "$ORIG_DIR"
    exit 1
}

# Εύρεση του φακέλου του script
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE}")" &> /dev/null && pwd)
cd "$SCRIPT_DIR" || abort "Αποτυχία αλλαγής φακέλου στο $SCRIPT_DIR"

export ISAAC_ROS_WS="$SCRIPT_DIR/.."
echo "Το ISAAC_ROS_WS ορίστηκε στο: $ISAAC_ROS_WS"

# Ορίζουμε το config directory ΜΕΣΑ στον φάκελο του workspace
export ISAAC_ROS_CONFIG_DIR="$ISAAC_ROS_WS/.isaac-ros"
echo "Το ISAAC_ROS_CONFIG_DIR ορίστηκε στο: $ISAAC_ROS_CONFIG_DIR"

export DOCKER_ARGS_FILE="$ISAAC_ROS_WS/.isaac-ros/isaac_ros_dev-dockerargs"

# 1. Έλεγχος αν υπάρχει το NVIDIA Container Toolkit
if ! command -v nvidia-ctk &> /dev/null; then
    abort "Λείπει το nvidia-container-toolkit. Εγκαταστήστε το στον host."
fi

# 2. Έλεγχος αν το Isaac ROS CLI είναι εγκατεστημένο και εύρεση της διαδρομής του
ISAAC_ROS_PATH=$(command -v isaac-ros)
if [ -z "$ISAAC_ROS_PATH" ]; then
    abort "Λείπει το isaac-ros-cli. Εγκαταστήστε το στον host."
fi

# 5. Εκκίνηση του Isaac ROS container
echo "Εκκίνηση του isaac-ros activate --build-local..."
isaac-ros activate --build-local|| abort "Αποτυχία εκκίνησης του isaac-ros activate --build-local"

# Επιστροφή στον αρχικό φάκελο
cd "$ORIG_DIR"
