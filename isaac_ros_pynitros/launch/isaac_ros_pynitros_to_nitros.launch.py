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
"""
Launch file for PyNITROS and NITROS interop.

This launch file use one PyNITROS image forward node
and one NITROS bridge node to demonstrate the zero-copy interop PyNITROS and NITROS.
"""

import launch

from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
import launch_testing.actions


def launch_setup(context):
    rosbag_path = LaunchConfiguration('rosbag_path').perform(context)

    rosbag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', rosbag_path, '--loop', '--remap',
             '/hawk_0_left_rgb_image:=/pynitros_input_msg'],
        output='screen')

    # Image forward node implemented with PyNITROS
    pynitros_image_forward_node = Node(
        name='pynitros_image_forward_node',
        package='isaac_ros_pynitros',
        executable='pynitros_image_forward_node',
        parameters=[{
            'enable_ros_subscribe': True,
            'enable_ros_publish': False
        }],
        output='screen'
    )

    # NITROS bridge node converts NITROS Bridge msg to NITROS msg
    nitros_bridge_node = ComposableNode(
        name='nitros_bridge_node',
        package='isaac_ros_nitros_bridge_ros2',
        plugin='nvidia::isaac_ros::nitros_bridge::ImageConverterNode',
        remappings=[
            ('ros2_input_bridge_image', 'pynitros_output_msg'),
            ('ros2_output_image', 'output'),
        ])

    container = ComposableNodeContainer(
        name='ros2_converter_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[nitros_bridge_node],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )
    return [rosbag_play, pynitros_image_forward_node, container]


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
