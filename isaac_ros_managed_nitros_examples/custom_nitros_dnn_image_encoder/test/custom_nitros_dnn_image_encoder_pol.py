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
import os
import pathlib
import time

from isaac_ros_tensor_list_interfaces.msg import TensorList
from isaac_ros_test import IsaacROSBaseTest, JSONConversion
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import pytest
import rclpy
from sensor_msgs.msg import Image


@pytest.mark.rostest
def generate_test_description():
    image_encoder_node = ComposableNode(
        name='custom_image_encoder',
        package='custom_nitros_dnn_image_encoder',
        plugin='custom_nitros_dnn_image_encoder::ImageEncoderNode',
        parameters=[{
            'input_image_height': 460,
            'input_image_width': 460
        }],
        namespace=CustomDNNImageEncoderPOLTest.generate_namespace(),
       )

    return CustomDNNImageEncoderPOLTest.generate_test_description([
        ComposableNodeContainer(
            name='custom_nitros_dnn_image_encoder_pol_test_container',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[image_encoder_node],
            namespace=CustomDNNImageEncoderPOLTest.generate_namespace(),
            output='screen',
            arguments=['--ros-args', '--log-level', 'info']
        )
    ])


class CustomDNNImageEncoderPOLTest(IsaacROSBaseTest):

    filepath = pathlib.Path(os.path.dirname(__file__))

    @IsaacROSBaseTest.for_each_test_case()
    def test_message_passthrough(self, test_folder):
        """
        Test DNN Image encoding.

        Test that the a image can be encoded into a GPU tensor using Managed
        Nitros and cvCUDA operations.
        """
        TIMEOUT = 10
        received_messages = {}

        self.generate_namespace_lookup(['image', 'encoded_tensor'])

        image_pub = self.node.create_publisher(
            Image, self.namespaces['image'], self.DEFAULT_QOS)

        subs = self.create_logging_subscribers([('encoded_tensor', TensorList)], received_messages)

        try:
            input_image = JSONConversion.load_image_from_json(test_folder / 'image.json')
            # Wait at most TIMEOUT seconds for subscriber to respond
            end_time = time.time() + TIMEOUT
            done = False

            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=(0.1))
                input_image.header.stamp.sec = 123456
                input_image.header.stamp.nanosec = 789101112
                input_image.header.frame_id = 'cuda_image'
                image_pub.publish(input_image)

                if 'encoded_tensor' in received_messages:
                    done = True
                    break

            self.assertTrue(done, 'Appropriate output not received')
            output_tensor_list = received_messages['encoded_tensor']
            self.assertEqual(
                len(output_tensor_list.tensors), 1, 'Did not receive exactly 1 output tensor')
            self.assertEqual(
                output_tensor_list.tensors[0].shape.rank, 4, 'Rank mismatch.')
            self.assertEqual(
                output_tensor_list.tensors[0].shape.dims[0], 1, 'Batch size mismatch.')
            self.assertEqual(
                output_tensor_list.tensors[0].shape.dims[1], 3, 'No of channel mismatch.')
            self.assertEqual(
                output_tensor_list.tensors[0].shape.dims[2], 480, 'Tensor height mismatch.')
            self.assertEqual(
                output_tensor_list.tensors[0].shape.dims[3], 640, 'Tensor width mismatch.')

        finally:
            self.node.destroy_publisher(image_pub)
            self.node.destroy_subscription(subs)
