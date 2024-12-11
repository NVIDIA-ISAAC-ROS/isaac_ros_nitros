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

from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import launch_testing.actions


def launch_setup(context):
    pub_tensor_list_name = LaunchConfiguration('pub_tensor_list_name').perform(context)
    sub_tensor_list_name = LaunchConfiguration('sub_tensor_list_name').perform(context)

    ros2_converter = ComposableNode(
        name='ros2_converter',
        namespace='',
        package='isaac_ros_nitros_bridge_ros2',
        plugin='nvidia::isaac_ros::nitros_bridge::TensorListConverterNode',
        remappings=[
            ('ros2_output_bridge_tensor_list', 'ros1_input_bridge_tensor_list'),
            ('ros2_output_tensor_list', pub_tensor_list_name),
            ('ros2_input_bridge_tensor_list', 'pynitros_tensor_list'),
            ('ros2_input_tensor_list', sub_tensor_list_name),
        ])

    container = ComposableNodeContainer(
        name='ros2_converter_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[ros2_converter],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )
    return [container]


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            'pub_tensor_list_name',
            description='Remapped publish image name of NITROS Bridge ROS2'),
        DeclareLaunchArgument(
            'sub_tensor_list_name',
            description='Remapped subscribe image name of NITROS Bridge ROS2'),
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
