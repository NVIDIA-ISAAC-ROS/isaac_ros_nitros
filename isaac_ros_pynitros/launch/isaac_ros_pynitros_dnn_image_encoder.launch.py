# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
"""DNN image encoder launch file using PyNITROS."""

import launch

from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch_testing.actions


def launch_setup(context):
    rosbag_path = LaunchConfiguration('rosbag_path').perform(context)

    rosbag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', rosbag_path, '--loop'],
        output='screen')

    dnn_image_encoder = Node(name='dnn_image_encoder',
                             package='isaac_ros_pynitros',
                             executable='pynitros_dnn_image_encoder_node',
                             parameters=[{
                                'enable_ros_subscribe': True,
                                'enable_ros_publish': True,
                                'network_image_width': 960,
                                'network_image_height': 544,
                                'image_mean': [0.5, 0.5, 0.5],
                                'image_std': [0.5, 0.5, 0.5]
                             }],
                             remappings=[
                                ('pynitros_input_msg', '/image_rect'),
                                ('pynitros_output_msg', 'pynitros1_output_msg'),
                                ('pynitros_output_msg_ros', 'tensor_pub')
                             ],
                             output='screen')

    return [rosbag_play,
            dnn_image_encoder]


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            'rosbag_path',
            description='Remapped publish image name of NITROS Bridge ROS2'),
    ]

    nodes = launch_args + [OpaqueFunction(function=launch_setup)]
    return launch.LaunchDescription(
        nodes + [
            # Start tests after a fixed delay for node startup
            launch.actions.TimerAction(
                period=30.0,
                actions=[launch_testing.actions.ReadyToTest()])
        ]
    )
