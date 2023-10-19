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

"""Proof-of-Life test for the NitrosOdometry type adapter."""

import os
import pathlib
import time

from isaac_ros_test import IsaacROSBaseTest, JSONConversion

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import launch_testing

from nav_msgs.msg import Odometry

import pytest
import rclpy


@pytest.mark.rostest
def generate_test_description():
    """Generate launch description with all ROS 2 nodes for testing."""
    test_ns = IsaacROSNitrosOdometryTest.generate_namespace()
    container = ComposableNodeContainer(
        name='test_container',
        namespace='isaac_ros_nitros_container',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_nitros_odometry_type',
                plugin='nvidia::isaac_ros::nitros::NitrosOdometryForwardNode',
                name='NitrosOdometryForwardNode',
                namespace=test_ns,
                parameters=[{
                    'compatible_format': 'nitros_odometry'
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

    return IsaacROSNitrosOdometryTest.generate_test_description([
        container,
        launch.actions.TimerAction(
            period=2.5, actions=[launch_testing.actions.ReadyToTest()])
    ])


class IsaacROSNitrosOdometryTest(IsaacROSBaseTest):
    """Validate NitrosOdometry type adapter."""

    filepath = pathlib.Path(os.path.dirname(__file__))

    @IsaacROSBaseTest.for_each_test_case(subfolder='nitros_odometry')
    def test_nitros_odometry_type_conversions(self, test_folder) -> None:
        """Expect the tag from NitrosOdometry type to be compatible with source."""
        self.generate_namespace_lookup(['input', 'output'])
        received_messages = {}

        received_message_sub = self.create_logging_subscribers(
            subscription_requests=[('output', Odometry)],
            received_messages=received_messages
        )

        odometry_pub = self.node.create_publisher(
            Odometry, self.namespaces['input'], self.DEFAULT_QOS)

        try:
            odometry = self.load_odometry_from_json(
                test_folder / 'odometry.json')

            # Wait at most TIMEOUT seconds for subscriber to respond
            TIMEOUT = 10
            end_time = time.time() + TIMEOUT

            done = False
            while time.time() < end_time:

                odometry_pub.publish(odometry)

                rclpy.spin_once(self.node, timeout_sec=0.1)

                # If we have received a message on the output topic, break
                if 'output' in received_messages:
                    done = True
                    break

            self.assertTrue(done, "Didn't receive output on the output topic!")

            received_odometry = received_messages['output']

            # Only test for data passed through gxf
            self.assertEqual(odometry.pose.pose.position.x,
                             received_odometry.pose.pose.position.x)
            self.assertEqual(odometry.pose.pose.position.y,
                             received_odometry.pose.pose.position.y)
            self.assertEqual(odometry.pose.pose.orientation.x,
                             received_odometry.pose.pose.orientation.x)
            self.assertEqual(odometry.pose.pose.orientation.y,
                             received_odometry.pose.pose.orientation.y)
            self.assertEqual(odometry.pose.pose.orientation.z,
                             received_odometry.pose.pose.orientation.z)
            self.assertEqual(odometry.pose.pose.orientation.w,
                             received_odometry.pose.pose.orientation.w)
            self.assertEqual(odometry.twist.twist.linear.x,
                             received_odometry.twist.twist.linear.x)
            self.assertEqual(odometry.twist.twist.angular.z,
                             received_odometry.twist.twist.angular.z)
            print('The received odometry message has been verified successfully')
        finally:
            self.node.destroy_subscription(received_message_sub)
            self.node.destroy_publisher(odometry_pub)

    @staticmethod
    def load_odometry_from_json(
            json_filepath: pathlib.Path) -> Odometry:
        """
        Load a Odometry message from a JSON filepath.

        Parameters
        ----------
        json_filepath : Path
            The path to a JSON file containing the Odometry fields

        Returns
        -------
        Odometry
            Generated Odometry message

        """
        odometry_json = JSONConversion.load_from_json(
            json_filepath)

        odometry = Odometry()
        odometry.header.frame_id = odometry_json[
            'header']['frame_id']
        odometry.header.stamp.sec = odometry_json[
            'header']['stamp']['sec']
        odometry.header.stamp.nanosec = odometry_json[
            'header']['stamp']['nanosec']
        odometry.child_frame_id = odometry_json['child_frame_id']
        odometry.pose.pose.position.x = odometry_json[
            'pose']['pose']['position']['x']
        odometry.pose.pose.position.y = odometry_json[
            'pose']['pose']['position']['y']
        odometry.pose.pose.position.z = odometry_json[
            'pose']['pose']['position']['z']
        odometry.pose.pose.orientation.x = odometry_json[
            'pose']['pose']['orientation']['x']
        odometry.pose.pose.orientation.y = odometry_json[
            'pose']['pose']['orientation']['y']
        odometry.pose.pose.orientation.z = odometry_json[
            'pose']['pose']['orientation']['z']
        odometry.pose.pose.orientation.w = odometry_json[
            'pose']['pose']['orientation']['w']
        odometry.twist.twist.linear.x = odometry_json[
            'twist']['twist']['linear']['x']
        odometry.twist.twist.linear.y = odometry_json[
            'twist']['twist']['linear']['y']
        odometry.twist.twist.linear.z = odometry_json[
            'twist']['twist']['linear']['z']
        odometry.twist.twist.angular.x = odometry_json[
            'twist']['twist']['angular']['x']
        odometry.twist.twist.angular.y = odometry_json[
            'twist']['twist']['angular']['y']
        odometry.twist.twist.angular.z = odometry_json[
            'twist']['twist']['angular']['z']
        return odometry
