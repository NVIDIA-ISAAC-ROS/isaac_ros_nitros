#!/bin/bash
#
# Copyright (c) 2024, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.



CONFIG_DIR="/workspaces/isaac_ros_1-dev/src/isaac_ros_nitros_bridge/config/"
SCRIPT_DIR="/workspaces/isaac_ros_1-dev/src/isaac_ros_nitros_bridge/scripts/"

echo "Using ROS1 config file: ${CONFIG_DIR}${1}"
echo "Using ROS1 launch file: $2"

# Set ptrace scope to enable CUDA IPC
echo 0 | sudo tee /proc/sys/kernel/yama/ptrace_scope

if [ -z "$3" ] && [ -z "$4" ]; then
    python3 "${SCRIPT_DIR}isaac_ros_nitros_bridge_ros1.py" -c "${CONFIG_DIR}${1}" -l $2
else
    python3 "${SCRIPT_DIR}isaac_ros_nitros_bridge_ros1.py" -c "${CONFIG_DIR}${1}" -l $2 -s $3 -p $4
fi

tail -f /dev/null