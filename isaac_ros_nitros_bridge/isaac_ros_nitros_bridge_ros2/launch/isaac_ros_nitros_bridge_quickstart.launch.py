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

import launch

from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import launch_testing.actions


def launch_setup(context):
    rosbag_path = LaunchConfiguration('rosbag_path').perform(context)

    rosbag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', rosbag_path, '--loop', '--remap',
             '/image:=/r2b/ros2_input_image'],
        output='screen')

    ros2_converter = ComposableNode(
        name='ros2_converter',
        namespace='r2b',
        package='isaac_ros_nitros_bridge_ros2',
        plugin='nvidia::isaac_ros::nitros_bridge::ImageConverterNode',
        parameters=[{
            'num_blocks': 40
        }],
        remappings=[
            ('ros2_output_bridge_image', 'ros1_input_bridge_image'),
            ('ros2_input_bridge_image', 'ros1_output_bridge_image')
        ])

    container = ComposableNodeContainer(
        name='ros2_converter_container',
        namespace='r2b',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[ros2_converter],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )
    return [rosbag_play, container]


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            'rosbag_path',
            description='Path of the r2b rosbag')
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
