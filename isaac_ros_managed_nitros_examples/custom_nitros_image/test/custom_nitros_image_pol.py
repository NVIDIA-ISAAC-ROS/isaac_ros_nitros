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

from isaac_ros_test import IsaacROSBaseTest, JSONConversion
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import pytest
import rclpy
from sensor_msgs.msg import Image


@pytest.mark.rostest
def generate_test_description():
    image_builder_node = ComposableNode(
        name='image_builder',
        package='custom_nitros_image',
        plugin='custom_nitros_image::GpuImageBuilderNode',
        namespace=CustomNitrosImagePOLTest.generate_namespace(),
       )

    image_viewer_node = ComposableNode(
        name='image_viewer',
        package='custom_nitros_image',
        plugin='custom_nitros_image::GpuImageViewerNode',
        namespace=CustomNitrosImagePOLTest.generate_namespace(),
        )

    return CustomNitrosImagePOLTest.generate_test_description([
        ComposableNodeContainer(
            name='custom_nitros_image_pol_test_container',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[image_viewer_node, image_builder_node],
            namespace=CustomNitrosImagePOLTest.generate_namespace(),
            output='screen',
            arguments=['--ros-args', '--log-level', 'info']
        )
    ])


class CustomNitrosImagePOLTest(IsaacROSBaseTest):

    filepath = pathlib.Path(os.path.dirname(__file__))

    @IsaacROSBaseTest.for_each_test_case()
    def test_message_passthrough(self, test_folder):
        """
        Test image building and viewing.

        Test that the custom NITROS image built with GPU based buffer from the input CPU image and
        view (access) the GPU image and make a CPU copy and send the reconstructed image.
        """
        TIMEOUT = 10
        received_messages = {}

        self.generate_namespace_lookup(['image_input', 'image_output'])

        image_pub = self.node.create_publisher(
            Image, self.namespaces['image_input'], self.DEFAULT_QOS)

        subs = self.create_logging_subscribers([('image_output', Image)], received_messages)

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

                image_pub.publish(input_image)

                if 'image_output' in received_messages:
                    done = True
                    break

            self.assertTrue(done, 'Appropriate output not received')

            output_image = received_messages['image_output']

            self.assertEqual(input_image, output_image, 'Output image did not match input!')

        finally:
            self.node.destroy_publisher(image_pub)
            self.node.destroy_subscription(subs)
