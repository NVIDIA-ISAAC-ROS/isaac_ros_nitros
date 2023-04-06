# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

"""Proof-of-Life test for the NitrosCompressedImage type adapter."""

import random
import time


from isaac_ros_test import IsaacROSBaseTest

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import launch_testing

import pytest
import rclpy
from sensor_msgs.msg import CompressedImage


@pytest.mark.rostest
def generate_test_description():
    """Generate launch description with all ROS 2 nodes for testing."""
    test_ns = IsaacROSNitrosCompressedImageTest.generate_namespace()
    container = ComposableNodeContainer(
        name='compressed_image_container',
        namespace='isaac_ros_nitros_container',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_nitros_compressed_image_type',
                plugin='nvidia::isaac_ros::nitros::NitrosCompressedImageForwardNode',
                name='NitrosCompressedImageForwardNode',
                namespace=test_ns,
                parameters=[{
                    'compatible_format': 'nitros_compressed_image'
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

    return IsaacROSNitrosCompressedImageTest.generate_test_description([
        container,
        launch.actions.TimerAction(
            period=2.5, actions=[launch_testing.actions.ReadyToTest()])
    ])


class IsaacROSNitrosCompressedImageTest(IsaacROSBaseTest):
    """Validate NitrosCompressedImage type adapter."""

    def test_nitros_compressed_image_type_conversions(self) -> None:
        """Expect data received to be identical to source."""
        self.generate_namespace_lookup(['input', 'output'])
        received_messages = {}

        received_compressed_img_sub = self.create_logging_subscribers(
            subscription_requests=[('output', CompressedImage)],
            received_messages=received_messages
        )

        compressed_img_pub = self.node.create_publisher(
            CompressedImage, self.namespaces['input'], self.DEFAULT_QOS)

        try:
            compressed_img_msg = CompressedImage()
            compressed_img_msg.format = 'h264'
            compressed_img_msg.data = [random.randint(0, 255) for _ in range(100)]
            # Wait at most TIMEOUT seconds for subscriber to respond
            TIMEOUT = 2
            end_time = time.time() + TIMEOUT

            done = False
            while time.time() < end_time:
                timestamp = self.node.get_clock().now().to_msg()
                compressed_img_msg.header.stamp = timestamp

                compressed_img_pub.publish(compressed_img_msg)
                rclpy.spin_once(self.node, timeout_sec=0.1)

                # If we have received a message on the output topic, break
                if 'output' in received_messages:
                    done = True
                    break

            self.assertTrue(done, "Didn't receive output on output_image topic!")

            received_compressed_img = received_messages['output']

            self.assertEqual(len(compressed_img_msg.data), len(received_compressed_img.data),
                             'Source and received image sizes do not match: ' +
                             f'{len(compressed_img_msg.data)} \
                             != {len(received_compressed_img.data)}')

            print('Source and received images are identical.')
        finally:
            self.node.destroy_subscription(received_compressed_img_sub)
            self.node.destroy_publisher(compressed_img_pub)
