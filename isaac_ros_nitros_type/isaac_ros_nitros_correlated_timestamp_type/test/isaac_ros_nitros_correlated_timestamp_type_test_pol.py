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

"""Proof-of-Life test for the NitrosCorrelatedTimestamp type adapter."""

import os
import pathlib
import time

from isaac_ros_nova_interfaces.msg import CorrelatedTimestamp
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
    test_ns = IsaacROSNitrosCorrelatedTimestampTest.generate_namespace()
    container = ComposableNodeContainer(
        name='test_container',
        namespace='isaac_ros_nitros_container',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_nitros_correlated_timestamp_type',
                plugin='nvidia::isaac_ros::nitros::NitrosCorrelatedTimestampForwardNode',
                name='NitrosCorrelatedTimestampForwardNode',
                namespace=test_ns,
                parameters=[{
                    'compatible_format': 'nitros_correlated_timestamp'
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

    return IsaacROSNitrosCorrelatedTimestampTest.generate_test_description([
        container,
        launch.actions.TimerAction(
            period=2.5, actions=[launch_testing.actions.ReadyToTest()])
    ])


class IsaacROSNitrosCorrelatedTimestampTest(IsaacROSBaseTest):
    """Validate NitrosCorrelatedTimestamp type adapter."""

    filepath = pathlib.Path(os.path.dirname(__file__))

    @IsaacROSBaseTest.for_each_test_case(subfolder='nitros_correlated_timestamp')
    def test_nitros_correlated_timestamp_type_conversions(self, test_folder) -> None:
        """Expect the tag from NitrosCorrelatedTimestamp type to be compatible with source."""
        self.generate_namespace_lookup(['input', 'output'])
        received_messages = {}

        received_message_sub = self.create_logging_subscribers(
            subscription_requests=[('output', CorrelatedTimestamp)],
            received_messages=received_messages
        )

        correlated_timestamp_pub = self.node.create_publisher(
            CorrelatedTimestamp, self.namespaces['input'], self.DEFAULT_QOS)

        try:
            correlated_timestamp = self.load_correlated_timestamp_from_json(
                test_folder / 'correlated_timestamp.json')

            # Wait at most TIMEOUT seconds for subscriber to respond
            TIMEOUT = 10
            end_time = time.time() + TIMEOUT

            done = False
            while time.time() < end_time:
                timestamp = self.node.get_clock().now().to_msg()
                correlated_timestamp.header.stamp = timestamp

                correlated_timestamp_pub.publish(correlated_timestamp)

                rclpy.spin_once(self.node, timeout_sec=0.1)

                # If we have received a message on the output topic, break
                if 'output' in received_messages:
                    done = True
                    break

            self.assertTrue(done, "Didn't receive output on the output topic!")

            received_correlated_timestamp = received_messages['output']

            # Only test for data passed through gxf
            self.assertEqual(correlated_timestamp.header.stamp.sec,
                             received_correlated_timestamp.header.stamp.sec)
            self.assertEqual(correlated_timestamp.header.stamp.nanosec,
                             received_correlated_timestamp.header.stamp.nanosec)
            self.assertEqual(correlated_timestamp.phc_val,
                             received_correlated_timestamp.phc_val)
            self.assertEqual(correlated_timestamp.tsc_val,
                             received_correlated_timestamp.tsc_val)
            self.assertEqual(correlated_timestamp.phc2_val,
                             received_correlated_timestamp.phc2_val)
            self.assertEqual(correlated_timestamp.sys_val,
                             received_correlated_timestamp.sys_val)
            self.assertEqual(correlated_timestamp.phc_latency,
                             received_correlated_timestamp.phc_latency)
            print('The received correlated_timestamp message has been verified successfully')
        finally:
            self.node.destroy_subscription(received_message_sub)
            self.node.destroy_publisher(correlated_timestamp_pub)

    @staticmethod
    def load_correlated_timestamp_from_json(
            json_filepath: pathlib.Path) -> CorrelatedTimestamp:
        """
        Load a CorrelatedTimestamp message from a JSON filepath.

        Parameters
        ----------
        json_filepath : Path
            The path to a JSON file containing the CorrelatedTimestamp fields

        Returns
        -------
        CorrelatedTimestamp
            Generated CorrelatedTimestamp message

        """
        correlated_timestamp_json = JSONConversion.load_from_json(
            json_filepath)

        correlated_timestamp = CorrelatedTimestamp()
        correlated_timestamp.header.frame_id = correlated_timestamp_json[
            'header']['frame_id']
        correlated_timestamp.header.stamp.sec = correlated_timestamp_json[
            'header']['stamp']['sec']
        correlated_timestamp.header.stamp.nanosec = correlated_timestamp_json[
            'header']['stamp']['nanosec']
        correlated_timestamp.phc_val = correlated_timestamp_json['phc_val']
        correlated_timestamp.tsc_val = correlated_timestamp_json['tsc_val']
        correlated_timestamp.phc2_val = correlated_timestamp_json['phc2_val']
        correlated_timestamp.sys_val = correlated_timestamp_json['sys_val']
        correlated_timestamp.phc_latency = correlated_timestamp_json['phc_latency']
        return correlated_timestamp
