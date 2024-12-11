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
"""Proof-of-Life test for the PyNITROS tensor list type."""

import os
import pathlib
import subprocess
import time

from isaac_ros_tensor_list_interfaces.msg import Tensor, TensorList, TensorShape
from isaac_ros_test import IsaacROSBaseTest
import launch

from launch_ros.actions import Node
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
        IsaacROSPyNitrosTest.skip_test = False
        pynitros_tensor_list_forward_node_1 = Node(
            name='pynitros_tensor_list_forward_node_1',
            package='isaac_ros_pynitros',
            namespace='pynitros',
            executable='pynitros_tensor_list_forward_node',
            parameters=[{
                'enable_ros_subscribe': True,
                'enable_ros_publish': False
            }],
            remappings=[('pynitros_input_msg', 'pynitros1_input_msg'),
                        ('pynitros_output_msg', 'pynitros1_output_msg'),
                        ('pynitros_output_msg_ros', 'pynitros1_output_msg_ros')],
            output='screen')

        pynitros_tensor_list_forward_node_2 = Node(
            name='pynitros_tensor_list_forward_node_2',
            package='isaac_ros_pynitros',
            executable='pynitros_tensor_list_forward_node',
            namespace='pynitros',
            parameters=[{
                'enable_ros_subscribe': False,
                'enable_ros_publish': True
            }],
            remappings=[('pynitros_input_msg', 'pynitros1_output_msg'),
                        ('pynitros_output_msg', 'pynitros2_output_msg'),
                        ('pynitros_output_msg_ros', 'pynitros2_output_msg_ros')],
            output='screen')

        return IsaacROSPyNitrosTest.generate_test_description([
            pynitros_tensor_list_forward_node_1, pynitros_tensor_list_forward_node_2,
            launch.actions.TimerAction(period=2.5, actions=[launch_testing.actions.ReadyToTest()])
        ])
    else:
        IsaacROSPyNitrosTest.skip_test = True
        return IsaacROSPyNitrosTest.generate_test_description(
            [launch_testing.actions.ReadyToTest()])


class IsaacROSPyNitrosTest(IsaacROSBaseTest):
    """Validate Nitros Bridge on Tensor List type."""

    filepath = pathlib.Path(os.path.dirname(__file__))

    @IsaacROSBaseTest.for_each_test_case(subfolder='nitros_image')
    def test_pynitros_tensor_list(self, test_folder) -> None:
        if self.skip_test:
            self.skipTest('No ptrace permission! Skipping test.')
        else:
            IsaacROSPyNitrosTest.DEFAULT_NAMESPACE = 'pynitros'
            self.generate_namespace_lookup(['pynitros1_input_msg', 'pynitros2_output_msg_ros'])
            received_messages = {}

            received_tensor_list_sub = self.create_logging_subscribers(
                subscription_requests=[('pynitros2_output_msg_ros', TensorList)],
                received_messages=received_messages)

            tensor_list_pub = self.node.create_publisher(TensorList,
                                                         self.namespaces['pynitros1_input_msg'],
                                                         self.DEFAULT_QOS)

            try:
                DATA_TYPE = 9
                INPUT_TENSOR_DIMENSIONS = [1, 3, 100, 100]
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
                    if 'pynitros2_output_msg_ros' in received_messages:
                        done = True
                        break

                self.assertTrue(done, "Didn't receive output on output tensor list topic!")

                received_tensor_list = received_messages['pynitros2_output_msg_ros']
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
                                         'Source and received tensor pixels do not match')

            finally:
                self.node.destroy_subscription(received_tensor_list_sub)
                self.node.destroy_publisher(tensor_list_pub)
