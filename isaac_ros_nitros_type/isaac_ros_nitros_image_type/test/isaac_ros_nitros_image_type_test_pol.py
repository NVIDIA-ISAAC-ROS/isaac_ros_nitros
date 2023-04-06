# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

"""Proof-of-Life test for the NitrosImage type adapter."""

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
    test_ns = IsaacROSNitrosImageTest.generate_namespace()
    container = ComposableNodeContainer(
        name='image_container',
        namespace='isaac_ros_nitros_container',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_nitros_image_type',
                plugin='nvidia::isaac_ros::nitros::NitrosImageForwardNode',
                name='NitrosImageForwardNode',
                namespace=test_ns,
                parameters=[{
                    'compatible_format': 'nitros_image_bgr8'
                }],
                remappings=[
                    (test_ns+'/topic_forward_input', test_ns+'/image'),
                    (test_ns+'/topic_forward_output', test_ns+'/output_image'),
                ]
            ),
        ],
        output='both',
        arguments=['--ros-args', '--log-level', 'info'],
    )

    return IsaacROSNitrosImageTest.generate_test_description([
        container,
        launch.actions.TimerAction(
            period=2.5, actions=[launch_testing.actions.ReadyToTest()])
    ])


class IsaacROSNitrosImageTest(IsaacROSBaseTest):
    """Validate NitrosImage type adapter."""

    filepath = pathlib.Path(os.path.dirname(__file__))

    @IsaacROSBaseTest.for_each_test_case(subfolder='nitros_image')
    def test_nitros_image_ros_type_conversions(self, test_folder) -> None:
        """Expect the image received from NitrosImage type conversion to be identical to source."""
        self.generate_namespace_lookup(['image', 'output_image'])
        received_messages = {}

        received_image_sub = self.create_logging_subscribers(
            subscription_requests=[('output_image', Image)],
            received_messages=received_messages
        )

        image_pub = self.node.create_publisher(
            Image, self.namespaces['image'], self.DEFAULT_QOS)

        try:
            image = JSONConversion.load_image_from_json(
                test_folder / 'image.json')

            # Wait at most TIMEOUT seconds for subscriber to respond
            TIMEOUT = 2
            end_time = time.time() + TIMEOUT

            done = False
            while time.time() < end_time:
                timestamp = self.node.get_clock().now().to_msg()
                image.header.stamp = timestamp

                image_pub.publish(image)
                rclpy.spin_once(self.node, timeout_sec=0.1)

                # If we have received a message on the output topic, break
                if 'output_image' in received_messages:
                    done = True
                    break

            self.assertTrue(done, "Didn't receive output on output_image topic!")

            received_image = received_messages['output_image']

            print(f'Source image data size: {len(image.data)}')
            print(f'Received image data size: {len(received_image.data)}')

            self.assertEqual(len(image.data), len(received_image.data),
                             'Source and received image sizes do not match: ' +
                             f'{len(image.data)} != {len(received_image.data)}')

            for i in range(100):
                self.assertEqual(image.data[i], received_image.data[i],
                                 'Souce and received image pixels do not match')

            # Make sure that the source and received images are the same
            self.assertEqual(received_image.height, image.height,
                             'Source and received image heights do not match: ' +
                             f'{image.height} != {received_image.height}')
            self.assertEqual(received_image.width, image.width,
                             'Source and received image widths do not match: ' +
                             f'{image.width} != {received_image.width}')

            print('Source and received images are identical.')
        finally:
            self.node.destroy_subscription(received_image_sub)
            self.node.destroy_publisher(image_pub)
