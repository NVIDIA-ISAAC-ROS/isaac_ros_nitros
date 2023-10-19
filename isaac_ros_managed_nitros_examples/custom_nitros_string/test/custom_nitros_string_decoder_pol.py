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

from isaac_ros_tensor_list_interfaces.msg import Tensor, TensorList, TensorShape
from isaac_ros_test import IsaacROSBaseTest
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import pytest
import rclpy
from std_msgs.msg import String


@pytest.mark.rostest
def generate_test_description():
    decoder_node = ComposableNode(
        name='decoder',
        package='custom_nitros_string',
        plugin='custom_nitros_string::StringDecoderNode',
        namespace=CustomNitrosStringDecoderPOLTest.generate_namespace(),
        parameters=[{
            'tensor_name': 'pol_tensor'
        }])

    return CustomNitrosStringDecoderPOLTest.generate_test_description([
        ComposableNodeContainer(
            name='custom_nitros_string_pol_test_container',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[decoder_node],
            namespace=CustomNitrosStringDecoderPOLTest.generate_namespace(),
            output='screen'
        )
    ])


class CustomNitrosStringDecoderPOLTest(IsaacROSBaseTest):

    def test_string_decoding(self):
        """
        Test string decoding.

        Test that the custom NITROS string Decoder correctly decodes the input tensor.
        """
        TIMEOUT = 300
        received_messages = {}

        self.generate_namespace_lookup(['string_output', 'encoded_tensor'])

        tensor_pub = self.node.create_publisher(
            TensorList, self.namespaces['encoded_tensor'], self.DEFAULT_QOS)

        subs = self.create_logging_subscribers(
            [('string_output', String)], received_messages)

        try:
            # Construct test tensor list
            DATA_TYPE = 2  # uint8_t
            INPUT_STRING = 'Hello, world!'
            INPUT_TENSOR_DIMENSIONS = [len(INPUT_STRING)]
            INPUT_TENSOR_NAME = 'pol_tensor'
            INPUT_TENSOR_STRIDES = [1]

            tensor_list = TensorList()
            tensor = Tensor()
            tensor_shape = TensorShape()

            tensor_shape.rank = len(INPUT_TENSOR_DIMENSIONS)
            tensor_shape.dims = INPUT_TENSOR_DIMENSIONS

            tensor.shape = tensor_shape
            tensor.name = INPUT_TENSOR_NAME
            tensor.data_type = DATA_TYPE
            tensor.strides = INPUT_TENSOR_STRIDES
            tensor.data = [ord(c) for c in INPUT_STRING]

            tensor_list.tensors = [tensor]

            end_time = time.time() + TIMEOUT
            done = False

            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=(0.1))

                tensor_pub.publish(tensor_list)

                if 'string_output' in received_messages:
                    done = True
                    break

            self.assertTrue(done, 'Appropriate output not received')

            output_string = received_messages['string_output']
            self.assertEqual(
                INPUT_STRING, output_string.data, 'Output string did not match input!')

        finally:
            self.node.destroy_publisher(tensor_pub)
            self.node.destroy_subscription(subs)
