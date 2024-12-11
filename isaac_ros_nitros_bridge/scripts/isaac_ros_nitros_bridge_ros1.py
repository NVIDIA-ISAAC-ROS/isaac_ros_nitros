# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import argparse
import os
import subprocess
import time

ROS1_WS_PATH = '/workspaces/isaac_ros_1-dev'
ROS2_WS_PATH = '/workspaces/isaac_ros-dev'

ROS1_INSTALL_PATH = 'install_isolated/setup.bash'
ROS1_BRIDGE_INSTALL_PATH = 'install/setup.bash'

DEFAULT_ROS1_LAUNCH_FILE = 'nitros_bridge_image_converter.launch'

parser = argparse.ArgumentParser(
    description='Start the NITROS Bridge ROS1 converter and ROS1 bridge.',
    formatter_class=argparse.ArgumentDefaultsHelpFormatter)

parser.add_argument("-c",
                    "--bridge_config_file",
                    type = str,
                    default = "nitros_bridge_image_converter.yaml",
                    help="Path to the bridge configuration file")

parser.add_argument("-l",
                    "--launch_file",
                    type = str,
                    default = DEFAULT_ROS1_LAUNCH_FILE,
                    help="ROS1 launch file name. Example: sample.launch")

parser.add_argument("-s",
                    "--sub_image_name",
                    type = str,
                    default = "/image",
                    help="ROS1 image converter subscriber topic name. Example: /image")

parser.add_argument("-p",
                    "--pub_image_name",
                    type = str,
                    default = "/ros1_output_image",
                    help="ROS1 image converter publisher topic name. Example: /ros1_output_image")

args = parser.parse_args()

roscore_cmd = 'source /opt/ros/noetic/setup.bash && roscore'
subprocess.Popen(
    roscore_cmd,
    env=os.environ,
    shell=True,
    executable='/bin/bash'
)
# Wait one second to ensure roscore is fully launched
time.sleep(1)

ros1_setup_path = os.path.join(ROS1_WS_PATH, ROS1_INSTALL_PATH)
stop_ros_nodes_cmd = f'source {ros1_setup_path} && rosnode kill -a'
subprocess.run(
    stop_ros_nodes_cmd,
    env=os.environ,
    shell=True,
    executable='/bin/bash'
)

ros1_launch_cmd = ""
if args.launch_file == DEFAULT_ROS1_LAUNCH_FILE:
    ros1_launch_cmd =  f'roslaunch isaac_ros_nitros_bridge_ros1 {DEFAULT_ROS1_LAUNCH_FILE} pub_image_name:={args.pub_image_name} sub_image_name:={args.sub_image_name}'
else:
    ros1_launch_cmd = f'roslaunch isaac_ros_nitros_bridge_ros1 {args.launch_file}'

ros1_converter_cmd = f'source {ros1_setup_path} \
    && rosparam load {args.bridge_config_file} \
    && {ros1_launch_cmd}'
subprocess.Popen(
    ros1_converter_cmd,
    env=os.environ,
    shell=True,
    executable='/bin/bash'
)

ros1_bridge_setup_path = os.path.join(ROS2_WS_PATH, ROS1_BRIDGE_INSTALL_PATH)
ros1_bridge_cmd = f'source {ros1_bridge_setup_path} && ros2 run ros1_bridge parameter_bridge'
subprocess.Popen(
    ros1_bridge_cmd,
    env=os.environ,
    shell=True,
    executable='/bin/bash'
)
