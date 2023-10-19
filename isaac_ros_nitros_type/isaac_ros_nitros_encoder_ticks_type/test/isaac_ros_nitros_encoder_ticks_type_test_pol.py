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

"""Proof-of-Life test for the NitrosEncoderTicks type adapter."""

import os
import pathlib
import time

from isaac_ros_nova_interfaces.msg import EncoderTicks
from isaac_ros_test import IsaacROSBaseTest, JSONConversion

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import launch_testing

import pytest
import rclpy


@pytest.mark.rostest
def generate_test_description():
    """Generate launch description with all ROS 2 nodes for testing."""
    test_ns = IsaacROSNitrosEncoderTicksTest.generate_namespace()
    container = ComposableNodeContainer(
        name='test_container',
        namespace='isaac_ros_nitros_container',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_nitros_encoder_ticks_type',
                plugin='nvidia::isaac_ros::nitros::NitrosEncoderTicksForwardNode',
                name='NitrosEncoderTicksForwardNode',
                namespace=test_ns,
                parameters=[{
                    'compatible_format': 'nitros_encoder_ticks'
                }],
                remappings=[
                    (test_ns+'/topic_forward_input', test_ns+'/input'),
                    (test_ns+'/topic_forward_output', test_ns+'/output'),
                ]
            ),
        ],
        output='both',
        arguments=['--ros-args', '--log-level', 'NitrosEncoderTicks:=debug'],
    )

    return IsaacROSNitrosEncoderTicksTest.generate_test_description([
        container,
        launch.actions.TimerAction(
            period=2.5, actions=[launch_testing.actions.ReadyToTest()])
    ])


class IsaacROSNitrosEncoderTicksTest(IsaacROSBaseTest):
    """Validate NitrosEncoderTicks type adapter."""

    filepath = pathlib.Path(os.path.dirname(__file__))

    @IsaacROSBaseTest.for_each_test_case(subfolder='nitros_encoder_ticks')
    def test_nitros_encoder_ticks_type_conversions(self, test_folder) -> None:
        """Expect the message from NitrosEncoderTicks type to be compatible with source."""
        self.generate_namespace_lookup(['input', 'output'])
        received_messages = {}

        received_message_sub = self.create_logging_subscribers(
            subscription_requests=[('output', EncoderTicks)],
            received_messages=received_messages
        )

        encoder_ticks_pub = self.node.create_publisher(
            EncoderTicks, self.namespaces['input'], self.DEFAULT_QOS)

        try:
            encoder_ticks = self.load_encoder_ticks_from_json(test_folder / 'encoder_ticks.json')

            # Wait at most TIMEOUT seconds for subscriber to respond
            TIMEOUT = 10
            end_time = time.time() + TIMEOUT

            done = False
            while time.time() < end_time:
                timestamp = self.node.get_clock().now().to_msg()
                encoder_ticks.header.stamp = timestamp

                encoder_ticks_pub.publish(encoder_ticks)

                rclpy.spin_once(self.node, timeout_sec=0.1)

                # If we have received a message on the output topic, break
                if 'output' in received_messages:
                    done = True
                    break

            self.assertTrue(done, "Didn't receive output on the output topic!")

            received_encoder_ticks = received_messages['output']

            # Only test for data passed through gxf
            # Header
            self.assertEqual(encoder_ticks.header.stamp.sec,
                             received_encoder_ticks.header.stamp.sec)
            self.assertEqual(encoder_ticks.header.stamp.nanosec,
                             received_encoder_ticks.header.stamp.nanosec)
            self.assertEqual(encoder_ticks.header.frame_id,
                             received_encoder_ticks.header.frame_id)
            # Values
            self.assertEqual(encoder_ticks.left_ticks,
                             received_encoder_ticks.left_ticks)
            self.assertEqual(encoder_ticks.right_ticks,
                             received_encoder_ticks.right_ticks)
            self.assertEqual(encoder_ticks.encoder_timestamp,
                             received_encoder_ticks.encoder_timestamp)
            print('The received encoder_ticks message has been verified successfully')
        finally:
            self.node.destroy_subscription(received_message_sub)
            self.node.destroy_publisher(encoder_ticks_pub)

    @staticmethod
    def load_encoder_ticks_from_json(
            json_filepath: pathlib.Path) -> EncoderTicks:
        """
        Load a EncoderTicks message from a JSON filepath.

        Parameters
        ----------
        json_filepath : Path
            The path to a JSON file containing the EncoderTicks fields

        Returns
        -------
        EncoderTicks
            Generated EncoderTicks message

        """
        encoder_ticks_json = JSONConversion.load_from_json(
            json_filepath)

        encoder_ticks = EncoderTicks()
        encoder_ticks.header.frame_id = encoder_ticks_json[
            'header']['frame_id']
        encoder_ticks.header.stamp.sec = encoder_ticks_json[
            'header']['stamp']['sec']
        encoder_ticks.header.stamp.nanosec = encoder_ticks_json[
            'header']['stamp']['nanosec']
        encoder_ticks.left_ticks = encoder_ticks_json['left_ticks']
        encoder_ticks.right_ticks = encoder_ticks_json['right_ticks']
        encoder_ticks.encoder_timestamp = encoder_ticks_json['encoder_timestamp']
        return encoder_ticks
