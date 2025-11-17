#!/bin/bash

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Start PAROL6 with ROS2 integration
#
# Usage:
#   ./start_parol6_ros2.sh [sim|real] [namespace]
#
# Examples:
#   ./start_parol6_ros2.sh sim /parol6          # Start in simulation mode
#   ./start_parol6_ros2.sh real /parol6         # Start with real robot

set -e  # Exit on error

# Default values
MODE="${1:-sim}"
NAMESPACE="${2:-/parol6}"
USE_RVIZ="${3:-true}"

echo "=========================================="
echo "Starting PAROL6 ROS2 Integration"
echo "=========================================="
echo "Mode: $MODE"
echo "Namespace: $NAMESPACE"
echo "Use RViz: $USE_RVIZ"
echo "=========================================="

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "ERROR: ROS2 is not sourced!"
    echo "Please source your ROS2 installation:"
    echo "  source /opt/ros/<distro>/setup.bash"
    exit 1
fi

echo "ROS2 Distro: $ROS_DISTRO"

# Check if in simulation or real mode
USE_SIM="false"
if [ "$MODE" == "sim" ]; then
    USE_SIM="true"
    echo "Running in SIMULATION mode"
else
    echo "Running in REAL robot mode"

    # Check if robot is connected
    if [ ! -e "/dev/ttyUSB0" ] && [ ! -e "/dev/ttyACM0" ]; then
        echo "WARNING: No robot detected at /dev/ttyUSB0 or /dev/ttyACM0"
        echo "Make sure your robot is connected!"
        read -p "Continue anyway? (y/n) " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    fi
fi

# Build ROS2 adapter if not already built
if [ ! -d "../../ros2_adapter/build" ]; then
    echo "Building ROS2 adapter..."
    cd ../../ros2_adapter
    pip install -e .
    cd -
fi

# Source the workspace
if [ -d "../../ros2_adapter/install/setup.bash" ]; then
    source ../../ros2_adapter/install/setup.bash
fi

# Launch the nodes
echo "Launching PAROL6 ROS2 nodes..."
ros2 launch ros2_adapter/launch/parol6_bringup.launch.py \
    use_sim:=$USE_SIM \
    namespace:=$NAMESPACE \
    use_rviz:=$USE_RVIZ

echo "PAROL6 ROS2 nodes stopped."
