# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
import time

from isaac_ros_tensor_list_interfaces.msg import TensorList
from isaac_ros_test import IsaacROSBaseTest
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import pytest
import rclpy
from std_msgs.msg import String


@pytest.mark.rostest
def generate_test_description():
    encoder_node = ComposableNode(
        name='encoder',
        package='custom_nitros_string',
        plugin='custom_nitros_string::StringEncoderNode',
        namespace=CustomNitrosStringEncoderPOLTest.generate_namespace(),
        parameters=[{
            'tensor_name': 'pol_tensor'
        }])

    return CustomNitrosStringEncoderPOLTest.generate_test_description([
        ComposableNodeContainer(
            name='custom_nitros_string_pol_test_container',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[encoder_node],
            namespace=CustomNitrosStringEncoderPOLTest.generate_namespace(),
            output='screen'
        )
    ])


class CustomNitrosStringEncoderPOLTest(IsaacROSBaseTest):

    def test_string_encoding(self):
        """
        Test string encoding.

        Test that the custom NITROS string encoder correctly encodes the input string.
        """
        TIMEOUT = 300
        received_messages = {}

        self.generate_namespace_lookup(['string_input', 'encoded_tensor'])

        string_pub = self.node.create_publisher(
            String, self.namespaces['string_input'], self.DEFAULT_QOS)

        subs = self.create_logging_subscribers(
            [('encoded_tensor', TensorList)], received_messages)

        try:
            input_string_msg = String()
            input_string_msg.data = 'Hello, world!'

            end_time = time.time() + TIMEOUT
            done = False

            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=(0.1))

                string_pub.publish(input_string_msg)

                if 'encoded_tensor' in received_messages:
                    done = True
                    break

            self.assertTrue(done, 'Appropriate output not received')

            output_tensor_list = received_messages['encoded_tensor']
            self.assertEqual(
                len(output_tensor_list.tensors), 1, 'Did not receive exactly 1 output tensor')

            output_tensor = output_tensor_list.tensors[0]
            output_string = ''.join([chr(num) for num in output_tensor.data[:-1]])

            self.assertEqual(
                input_string_msg.data, output_string, 'Output string did not match input!')

        finally:
            self.node.destroy_publisher(string_pub)
            self.node.destroy_subscription(subs)
