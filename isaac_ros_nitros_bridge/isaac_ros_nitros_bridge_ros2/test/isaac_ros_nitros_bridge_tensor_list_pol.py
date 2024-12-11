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
Proof-of-Life test for interprocess NITROS bridge tensor list on ROS 2.

NITROSBridgeTensorListConverter1(NITROSTensorList->NITROSBridgeTensorList)
NITROSBridgeTensorListConverter2(NITROSBridgeTensorList->NITROSTensorList)
"""

import os
import pathlib
import subprocess
import time

from isaac_ros_tensor_list_interfaces.msg import Tensor, TensorList, TensorShape
from isaac_ros_test import IsaacROSBaseTest

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import launch_testing
import numpy as np

import pytest
import rclpy


@pytest.mark.rostest
def generate_test_description():
    """Generate launch description with all ROS 2 nodes for testing."""
    ptace_result = subprocess.run(['cat', '/proc/sys/kernel/yama/ptrace_scope'],
                                  stdout=subprocess.PIPE,
                                  stderr=subprocess.PIPE,
                                  text=True)
    pidfd_getfd_result = subprocess.run(['grep', 'pidfd_getfd', '/proc/kallsyms'],
                                        stdout=subprocess.PIPE,
                                        stderr=subprocess.PIPE,
                                        text=True)
    if (ptace_result.stdout.strip() == '0' and pidfd_getfd_result.returncode == 0):
        tensor_list_converter_1 = ComposableNode(
            name='ros2_converter',
            namespace='r2b',
            package='isaac_ros_nitros_bridge_ros2',
            plugin='nvidia::isaac_ros::nitros_bridge::TensorListConverterNode',
            parameters=[{
                'num_blocks': 5
            }],
            remappings=[
                ('ros2_output_bridge_tensor_list', 'ros2_converter_output'),
            ])

        tensor_list_converter_2 = ComposableNode(
            name='ros2_converter',
            namespace='r2b',
            package='isaac_ros_nitros_bridge_ros2',
            plugin='nvidia::isaac_ros::nitros_bridge::TensorListConverterNode',
            parameters=[{
                'num_blocks': 5
            }],
            remappings=[('ros2_input_bridge_tensor_list', 'ros2_converter_output')])

        tensor_list_converter_container_1 = ComposableNodeContainer(
            name='ros2_converter_1_container',
            namespace='r2b',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[tensor_list_converter_1],
            output='screen',
            arguments=['--ros-args', '--log-level', 'info'])

        tensor_list_converter_container_2 = ComposableNodeContainer(
            name='ros2_converter_2_container',
            namespace='r2b',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[tensor_list_converter_2],
            output='screen',
            arguments=['--ros-args', '--log-level', 'info'])

        return IsaacROSNitrosBridgeTest.generate_test_description([
            tensor_list_converter_container_1, tensor_list_converter_container_2,
            launch.actions.TimerAction(period=2.5, actions=[launch_testing.actions.ReadyToTest()])
        ])
    else:
        IsaacROSNitrosBridgeTest.skip_test = True
        return IsaacROSNitrosBridgeTest.generate_test_description(
            [launch_testing.actions.ReadyToTest()])


class IsaacROSNitrosBridgeTest(IsaacROSBaseTest):
    """Validate Nitros Bridge on TensorList type."""

    filepath = pathlib.Path(os.path.dirname(__file__))
    skip_test = False

    def test_nitros_bridge_tensor_list(self) -> None:
        """Expect the received tensor list to be identical to source."""
        if self.skip_test:
            self.skipTest('No ptrace permission! Skipping test.')
        else:
            IsaacROSNitrosBridgeTest.DEFAULT_NAMESPACE = 'r2b'
            self.generate_namespace_lookup(['ros2_input_tensor_list', 'ros2_output_tensor_list'])
            received_messages = {}

            received_tensor_list_sub = self.create_logging_subscribers(
                subscription_requests=[('ros2_output_tensor_list', TensorList)],
                received_messages=received_messages)

            tensor_list_pub = self.node.create_publisher(TensorList,
                                                         self.namespaces['ros2_input_tensor_list'],
                                                         self.DEFAULT_QOS)

            try:
                DATA_TYPE = 9
                INPUT_TENSOR_DIMENSIONS = [10, 3, 100, 100]
                INPUT_TENSOR_NAME = 'input'
                INPUT_TENSOR_STRIDE = 4

                tensor_list = TensorList()
                tensor = Tensor()
                tensor_shape = TensorShape()

                tensor_shape.rank = len(INPUT_TENSOR_DIMENSIONS)
                tensor_shape.dims = INPUT_TENSOR_DIMENSIONS

                tensor.shape = tensor_shape
                tensor.name = INPUT_TENSOR_NAME
                tensor.data_type = DATA_TYPE
                tensor.strides = []

                data_length = INPUT_TENSOR_STRIDE * np.prod(INPUT_TENSOR_DIMENSIONS)
                tensor.data = np.random.randint(256, size=data_length).tolist()

                tensor_list.tensors = [tensor, tensor]
                timestamp = self.node.get_clock().now().to_msg()
                tensor_list.header.stamp = timestamp

                # Wait at most TIMEOUT seconds for subscriber to respond
                TIMEOUT = 30
                end_time = time.time() + TIMEOUT

                done = False
                while time.time() < end_time:
                    tensor_list_pub.publish(tensor_list)
                    rclpy.spin_once(self.node, timeout_sec=0.1)

                    # If we have received a message on the output topic, break
                    if 'ros2_output_tensor_list' in received_messages:
                        done = True
                        break

                self.assertTrue(done, "Didn't receive output on output_tensor_list topic!")

                received_tensor_list = received_messages['ros2_output_tensor_list']
                self.assertEqual(len(tensor_list.tensors), len(received_tensor_list.tensors),
                                 'Source and received tensor list size do not match')

                for i in range(len(tensor_list.tensors)):
                    source_tensor = tensor_list.tensors[i]
                    received_tensor = received_tensor_list.tensors[i]
                    self.assertEqual(source_tensor.name, received_tensor.name,
                                     'Source and received tensor names do not match')
                    self.assertEqual(source_tensor.name, received_tensor.name,
                                     'Source and received tensor names do not match')
                    self.assertEqual(source_tensor.data_type, received_tensor.data_type,
                                     'Source and received tensor data types do not match')
                    self.assertEqual(source_tensor.shape.rank, received_tensor.shape.rank,
                                     'Source and received tensor ranks do not match')
                    self.assertEqual(source_tensor.shape.dims, received_tensor.shape.dims,
                                     'Source and received tensor dimensions do not match')
                    self.assertEqual(len(source_tensor.data), len(received_tensor.data),
                                     'Source and received tensor data do not match')
                    self.assertEqual(str(timestamp), str(received_tensor_list.header.stamp),
                                     'Timestamps do not match.')

                    for j in range(len(source_tensor.data)):
                        self.assertEqual(source_tensor.data[j], received_tensor.data[j],
                                         'Source and received image pixels do not match')

            finally:
                self.node.destroy_subscription(received_tensor_list_sub)
                self.node.destroy_publisher(tensor_list_pub)
