# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

"""Proof-of-Life test for the NitrosInt64 type adapter."""

import os
import pathlib
import time

from isaac_ros_test import IsaacROSBaseTest
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import pytest
import rclpy

from std_msgs.msg import Int64


@pytest.mark.rostest
def generate_test_description():
    """Generate launch description with all ROS 2 nodes for testing."""
    test_ns = IsaacROSNitrosInt64Test.generate_namespace()
    container = ComposableNodeContainer(
        name='test_container',
        namespace='isaac_ros_nitros_container',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_nitros_std_msg_type',
                plugin='nvidia::isaac_ros::nitros::NitrosInt64ForwardNode',
                name='NitrosInt64ForwardNode',
                namespace=test_ns,
                parameters=[{
                    'compatible_format': 'nitros_int64'
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

    return IsaacROSNitrosInt64Test.generate_test_description(
        [container],
        node_startup_delay=2.5
    )


class IsaacROSNitrosInt64Test(IsaacROSBaseTest):
    """Validate NitrosInt64 type adapter."""

    filepath = pathlib.Path(os.path.dirname(__file__))

    @IsaacROSBaseTest.for_each_test_case()
    def test_nitros_int64_type_conversions(self, test_folder) -> None:
        """Expect the int64 from NitrosInt64 type to be compatible with source."""
        self.generate_namespace_lookup(['input', 'output'])
        received_messages = {}

        received_message_sub = self.create_logging_subscribers(
            subscription_requests=[('output', Int64)],
            received_messages=received_messages
        )

        int64_msg_pub = self.node.create_publisher(
            Int64, self.namespaces['input'], self.DEFAULT_QOS)

        try:
            int64_msg = Int64()
            int64_msg.data = 42

            # Wait at most TIMEOUT seconds for subscriber to respond
            TIMEOUT = 2
            end_time = time.time() + TIMEOUT

            done = False
            while time.time() < end_time:
                int64_msg_pub.publish(int64_msg)

                rclpy.spin_once(self.node, timeout_sec=0.1)

                # If we have received a message on the output topic, break
                if 'output' in received_messages:
                    done = True
                    break

            self.assertTrue(done, "Didn't receive output on the output topic!")

            received_int64_msg = received_messages['output']

            self.assertEqual(received_int64_msg.data, 42, 'Number stored in Int64 does not match')

            print('The received int64 was verified successfully')
        finally:
            self.node.destroy_subscription(received_message_sub)
            self.node.destroy_publisher(int64_msg_pub)
