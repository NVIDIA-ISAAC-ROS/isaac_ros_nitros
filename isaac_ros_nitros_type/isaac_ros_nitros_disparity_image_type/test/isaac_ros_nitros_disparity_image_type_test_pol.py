# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

"""Proof-of-Life test for the NitrosDisparityImage type adapter."""

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

from stereo_msgs.msg import DisparityImage


@pytest.mark.rostest
def generate_test_description():
    """Generate launch description with all ROS 2 nodes for testing."""
    test_ns = IsaacROSDisparityImageTest.generate_namespace()
    container = ComposableNodeContainer(
        name='image_container',
        namespace='isaac_ros_nitros_container',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_nitros_disparity_image_type',
                plugin='nvidia::isaac_ros::nitros::NitrosDisparityImageForwardNode',
                name='NitrosDisparityImageForwardNode',
                namespace=test_ns,
                parameters=[{
                    'compatible_format': 'nitros_disparity_image_bgr8'
                }],
                remappings=[
                    (test_ns+'/topic_forward_input', test_ns+'/input'),
                    (test_ns+'/topic_forward_output', test_ns+'/output'),
                ]
            ),
        ],
        output='both',
        arguments=['--ros-args', '--log-level', 'error'],
    )

    return IsaacROSDisparityImageTest.generate_test_description([
        container,
        launch.actions.TimerAction(
            period=2.5, actions=[launch_testing.actions.ReadyToTest()])
    ])


class IsaacROSDisparityImageTest(IsaacROSBaseTest):
    """Validate NitrosDisparityImage type adapter."""

    filepath = pathlib.Path(os.path.dirname(__file__))

    @IsaacROSBaseTest.for_each_test_case(subfolder='nitros_disparity_image')
    def test_nitros_image_type_conversions(self, test_folder) -> None:
        """Expect the image received from NitrosImage type conversion to be identical to source."""
        self.generate_namespace_lookup(['input', 'output'])
        received_messages = {}

        received_image_sub = self.create_logging_subscribers(
            subscription_requests=[('output', DisparityImage)],
            received_messages=received_messages
        )

        image_pub = self.node.create_publisher(
            DisparityImage, self.namespaces['input'], self.DEFAULT_QOS)

        try:
            disparity_image = DisparityImage()
            disparity_image.image = JSONConversion.load_image_from_json(
                test_folder / 'image.json')
            disparity_image.f = 1.0
            disparity_image.t = 2.0
            disparity_image.min_disparity = 0.0
            disparity_image.max_disparity = 64.0
            # Wait at most TIMEOUT seconds for subscriber to respond
            TIMEOUT = 10
            end_time = time.time() + TIMEOUT

            done = False
            while time.time() < end_time:
                timestamp = self.node.get_clock().now().to_msg()
                disparity_image.header.stamp = timestamp

                image_pub.publish(disparity_image)
                rclpy.spin_once(self.node, timeout_sec=2)

                # If we have received a message on the output topic, break
                if 'output' in received_messages:
                    done = True
                    break

            self.assertTrue(done, "Didn't receive output on output_image topic!")

            received_image = received_messages['output']

            print(f'Source image data size: {len(disparity_image.image.data)}')
            print(f'Received image data size: {len(received_image.image.data)}')

            self.assertEqual(
                len(disparity_image.image.data), len(received_image.image.data),
                'Source and received image sizes do not match: ' +
                f'{len(disparity_image.image.data)} != {len(received_image.image.data)}')

            self.assertEqual(disparity_image.f, received_image.f,
                             'Source and received f value do not match: ' +
                             f'{disparity_image.f} != {received_image.f}')
            self.assertEqual(disparity_image.t, received_image.t,
                             'Source and received t value do not match: ' +
                             f'{disparity_image.t} != {received_image.t}')

            self.assertEqual(disparity_image.min_disparity, received_image.min_disparity,
                             'Source and received min_disparity do not match: ' +
                             f'{disparity_image.min_disparity} != {received_image.min_disparity}')
            self.assertEqual(disparity_image.max_disparity, received_image.max_disparity,
                             'Source and received max_disparity do not match: ' +
                             f'{disparity_image.max_disparity} != {received_image.max_disparity}')

            # Make sure that the source and received images are the same
            self.assertEqual(received_image.image.height, disparity_image.image.height,
                             'Source and received image heights do not match: ' +
                             f'{received_image.image.height} != {disparity_image.image.height}')
            self.assertEqual(received_image.image.width, disparity_image.image.width,
                             'Source and received image widths do not match: ' +
                             f'{received_image.image.width} != {disparity_image.image.width}')

            for i in range(len(disparity_image.image.data)):
                self.assertEqual(disparity_image.image.data[i], received_image.image.data[i],
                                 'Source and received image pixels do not match')
            print('Source and received images are identical.')
        finally:
            self.node.destroy_subscription(received_image_sub)
            self.node.destroy_publisher(image_pub)
