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
Proof-of-Life test for NITROS Bridge between ROS 1 and ROS 2.

Need to run together with the ROS Noeitc example container.
"""

import os
import pathlib
import time

from isaac_ros_test import IsaacROSBaseTest, JSONConversion

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import launch_testing

import pytest
import rclpy
from sensor_msgs.msg import Image


@pytest.mark.rostest
def generate_test_description():
    """Generate launch description with all ROS 2 nodes for testing."""
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

    return IsaacROSNitrosBridgeTest.generate_test_description([
        container,
        launch.actions.TimerAction(
            period=2.5, actions=[launch_testing.actions.ReadyToTest()])
    ])


class IsaacROSNitrosBridgeTest(IsaacROSBaseTest):
    """Validate Nitros Bridge on Image type."""

    filepath = pathlib.Path(os.path.dirname(__file__))

    @IsaacROSBaseTest.for_each_test_case(subfolder='nitros_image')
    def test_nitros_bridge_image(self, test_folder) -> None:
        """Expect the image received from NitrosImage type conversion to be identical to source."""
        IsaacROSNitrosBridgeTest.DEFAULT_NAMESPACE = 'r2b'
        self.generate_namespace_lookup(['ros2_input_image', 'ros2_output_image'])
        received_messages = {}

        received_image_sub = self.create_logging_subscribers(
            subscription_requests=[('ros2_output_image', Image)],
            received_messages=received_messages
        )

        image_pub = self.node.create_publisher(
            Image, self.namespaces['ros2_input_image'], self.DEFAULT_QOS)

        try:
            image = JSONConversion.load_image_from_json(
                test_folder / 'image.json')
            timestamp = self.node.get_clock().now().to_msg()
            image.header.stamp = timestamp

            # Wait at most TIMEOUT seconds for subscriber to respond
            TIMEOUT = 30
            end_time = time.time() + TIMEOUT

            done = False
            while time.time() < end_time:
                image_pub.publish(image)
                rclpy.spin_once(self.node, timeout_sec=0.1)

                # If we have received a message on the output topic, break
                if 'ros2_output_image' in received_messages:
                    done = True
                    break

            self.assertTrue(done, "Didn't receive output on output_image topic!")

            received_image = received_messages['ros2_output_image']

            print(f'Source image data size: {len(image.data)}')
            print(f'Received image data size: {len(received_image.data)}')

            self.assertEqual(str(timestamp), str(received_image.header.stamp),
                             'Timestamps do not match.')

            self.assertEqual(len(image.data), len(received_image.data),
                             'Source and received image sizes do not match: ' +
                             f'{len(image.data)} != {len(received_image.data)}')

            # Make sure that the source and received images are the same
            self.assertEqual(received_image.height, image.height,
                             'Source and received image heights do not match: ' +
                             f'{image.height} != {received_image.height}')
            self.assertEqual(received_image.width, image.width,
                             'Source and received image widths do not match: ' +
                             f'{image.width} != {received_image.width}')

            for i in range(len(image.data)):
                self.assertEqual(image.data[i], received_image.data[i],
                                 'Source and received image pixels do not match')

        finally:
            self.node.destroy_subscription(received_image_sub)
            self.node.destroy_publisher(image_pub)
