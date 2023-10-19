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

"""Proof-of-Life test for the NitrosTwist type adapter."""

import os
import pathlib
import time

from geometry_msgs.msg import Twist

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
    test_ns = IsaacROSNitrosTwistTest.generate_namespace()
    container = ComposableNodeContainer(
        name='test_container',
        namespace='isaac_ros_nitros_container',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_nitros_twist_type',
                plugin='nvidia::isaac_ros::nitros::NitrosTwistForwardNode',
                name='NitrosTwistForwardNode',
                namespace=test_ns,
                parameters=[{
                    'compatible_format': 'nitros_twist'
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

    return IsaacROSNitrosTwistTest.generate_test_description([
        container,
        launch.actions.TimerAction(
            period=2.5, actions=[launch_testing.actions.ReadyToTest()])
    ])


class IsaacROSNitrosTwistTest(IsaacROSBaseTest):
    """Validate NitrosTwist type adapter."""

    filepath = pathlib.Path(os.path.dirname(__file__))

    @IsaacROSBaseTest.for_each_test_case(subfolder='nitros_twist')
    def test_nitros_twist_type_conversions(self, test_folder) -> None:
        """Expect the tag from NitrosTwist type to be compatible with source."""
        self.generate_namespace_lookup(['input', 'output'])
        received_messages = {}

        received_message_sub = self.create_logging_subscribers(
            subscription_requests=[('output', Twist)],
            received_messages=received_messages
        )

        twist_pub = self.node.create_publisher(
            Twist, self.namespaces['input'], self.DEFAULT_QOS)

        try:
            twist = self.load_twist_from_json(test_folder / 'twist.json')

            # Wait at most TIMEOUT seconds for subscriber to respond
            TIMEOUT = 10
            end_time = time.time() + TIMEOUT

            done = False
            while time.time() < end_time:

                twist_pub.publish(twist)

                rclpy.spin_once(self.node, timeout_sec=0.1)

                # If we have received a message on the output topic, break
                if 'output' in received_messages:
                    done = True
                    break

            self.assertTrue(done, "Didn't receive output on the output topic!")

            received_twist = received_messages['output']

            # Only test for data passed through gxf
            self.assertEqual(twist.linear.x,
                             received_twist.linear.x)
            self.assertEqual(twist.angular.z,
                             received_twist.angular.z)
            print('The received twist message has been verified successfully')
        finally:
            self.node.destroy_subscription(received_message_sub)
            self.node.destroy_publisher(twist_pub)

    @staticmethod
    def load_twist_from_json(
            json_filepath: pathlib.Path) -> Twist:
        """
        Load a Twist message from a JSON filepath.

        Parameters
        ----------
        json_filepath : Path
            The path to a JSON file containing the Twist fields

        Returns
        -------
        Twist
            Generated Twist message

        """
        twist_json = JSONConversion.load_from_json(
            json_filepath)

        twist = Twist()
        twist.linear.x = twist_json['linear']['x']
        twist.linear.y = twist_json['linear']['y']
        twist.linear.z = twist_json['linear']['z']
        twist.angular.x = twist_json['angular']['x']
        twist.angular.y = twist_json['angular']['y']
        twist.angular.z = twist_json['angular']['z']
        return twist
