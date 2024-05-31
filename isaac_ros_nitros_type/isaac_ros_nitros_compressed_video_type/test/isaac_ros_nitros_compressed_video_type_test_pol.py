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

"""Proof-of-Life test for the NitrosCompressedVideo type adapter."""

import random
import time

from foxglove_msgs.msg import CompressedVideo
from isaac_ros_test import IsaacROSBaseTest

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import launch_testing

import pytest
import rclpy


@pytest.mark.rostest
def generate_test_description():
    """Generate launch description with all ROS 2 nodes for testing."""
    test_ns = IsaacROSNitrosCompressedVideoTest.generate_namespace()
    container = ComposableNodeContainer(
        name='compressed_video_container',
        namespace='isaac_ros_nitros_container',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_nitros_compressed_video_type',
                plugin='nvidia::isaac_ros::nitros::NitrosCompressedVideoForwardNode',
                name='NitrosCompressedVideoForwardNode',
                namespace=test_ns,
                parameters=[{
                    'compatible_format': 'nitros_compressed_video'
                }],
                remappings=[
                    (test_ns+'/topic_forward_input', test_ns+'/input'),
                    (test_ns+'/topic_forward_output', test_ns+'/output'),
                ]
            ),
        ],
        output='both',
        arguments=['--ros-args', '--log-level', 'info'],
    )

    return IsaacROSNitrosCompressedVideoTest.generate_test_description([
        container,
        launch.actions.TimerAction(
            period=2.5, actions=[launch_testing.actions.ReadyToTest()])
    ])


class IsaacROSNitrosCompressedVideoTest(IsaacROSBaseTest):
    """Validate NitrosCompressedVideo type adapter."""

    def test_nitros_compressed_video_type_conversions(self) -> None:
        """Expect data received to be identical to source."""
        self.generate_namespace_lookup(['input', 'output'])
        received_messages = {}

        received_compressed_video_sub = self.create_logging_subscribers(
            subscription_requests=[('output', CompressedVideo)],
            received_messages=received_messages
        )

        compressed_video_pub = self.node.create_publisher(
            CompressedVideo, self.namespaces['input'], self.DEFAULT_QOS)

        try:
            compressed_video_msg = CompressedVideo()
            compressed_video_msg.format = 'h264'
            compressed_video_msg.data = [random.randint(0, 255) for _ in range(100)]
            # Wait at most TIMEOUT seconds for subscriber to respond
            TIMEOUT = 2
            end_time = time.time() + TIMEOUT

            done = False
            while time.time() < end_time:
                timestamp = self.node.get_clock().now().to_msg()
                compressed_video_msg.timestamp = timestamp

                compressed_video_pub.publish(compressed_video_msg)
                rclpy.spin_once(self.node, timeout_sec=0.1)

                # If we have received a message on the output topic, break
                if 'output' in received_messages:
                    done = True
                    break

            self.assertTrue(done, 'Did not receive message on output topic!')

            received_compressed_video = received_messages['output']

            self.assertEqual(compressed_video_msg.format, received_compressed_video.format,
                             'Source and received video format do not match: ' +
                             f'{compressed_video_msg.format} \
                             != {received_compressed_video.format}')

            self.assertEqual(compressed_video_msg.data, received_compressed_video.data,
                             'Source and received video data do not match')

            print('Source and received videos are identical.')
        finally:
            self.node.destroy_subscription(received_compressed_video_sub)
            self.node.destroy_publisher(compressed_video_pub)
