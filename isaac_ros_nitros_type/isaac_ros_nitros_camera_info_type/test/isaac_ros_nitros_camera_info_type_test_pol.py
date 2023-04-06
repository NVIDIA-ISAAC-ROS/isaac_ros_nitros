# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

"""Proof-of-Life test for the NitrosCameraInfo type adapter."""

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
from sensor_msgs.msg import CameraInfo


@pytest.mark.rostest
def generate_test_description():
    """Generate launch description with all ROS 2 nodes for testing."""
    test_ns = IsaacROSNitrosCameraInfoTest.generate_namespace()
    container = ComposableNodeContainer(
        name='test_container',
        namespace='isaac_ros_nitros_container',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_nitros_camera_info_type',
                plugin='nvidia::isaac_ros::nitros::NitrosCameraInfoForwardNode',
                name='NitrosCameraInfoForwardNode',
                namespace=test_ns,
                parameters=[{
                    'compatible_format': 'nitros_camera_info'
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

    return IsaacROSNitrosCameraInfoTest.generate_test_description([
        container,
        launch.actions.TimerAction(
            period=2.5, actions=[launch_testing.actions.ReadyToTest()])
    ])


class IsaacROSNitrosCameraInfoTest(IsaacROSBaseTest):
    """Validate NitrosCameraInfo type adapter."""

    filepath = pathlib.Path(os.path.dirname(__file__))

    @IsaacROSBaseTest.for_each_test_case(subfolder='nitros_camera_info')
    def test_nitros_camera_info_type_conversions(self, test_folder) -> None:
        """Expect the camera info from NitrosCameraInfo type to be compatible with source."""
        self.generate_namespace_lookup(['input', 'output'])
        received_messages = {}

        received_message_sub = self.create_logging_subscribers(
            subscription_requests=[('output', CameraInfo)],
            received_messages=received_messages
        )

        camera_info_pub = self.node.create_publisher(
            CameraInfo, self.namespaces['input'], self.DEFAULT_QOS)

        try:
            camera_info = JSONConversion.load_camera_info_from_json(
                test_folder / 'camera_info.json')

            # Wait at most TIMEOUT seconds for subscriber to respond
            TIMEOUT = 2
            end_time = time.time() + TIMEOUT

            done = False
            while time.time() < end_time:
                timestamp = self.node.get_clock().now().to_msg()
                camera_info.header.stamp = timestamp

                camera_info_pub.publish(camera_info)

                rclpy.spin_once(self.node, timeout_sec=0.1)

                # If we have received a message on the output topic, break
                if 'output' in received_messages:
                    done = True
                    break

            self.assertTrue(done, "Didn't receive output on the output topic!")

            received_camera_info = received_messages['output']

            # Header
            # Frame ID is to be passed from NitrosImage to ROS message
            # self.assertEqual(camera_info.header.frame_id, received_camera_info.header.frame_id)

            # Size
            self.assertEqual(camera_info.height, received_camera_info.height)
            self.assertEqual(camera_info.width, received_camera_info.width)

            # Distortion model
            self.assertEqual(camera_info.distortion_model, received_camera_info.distortion_model)

            # D
            for i in range(min(len(camera_info.d), len(received_camera_info.d))):
                self.assertEqual(
                    round(camera_info.d[i], 2), round(received_camera_info.d[i], 2),
                    f'{i+1}th D value does not match')

            # K
            self.assertEqual(len(camera_info.k), len(received_camera_info.k))
            for i in range(min(len(camera_info.k), len(received_camera_info.k))):
                self.assertEqual(
                    round(camera_info.k[i], 2), round(received_camera_info.k[i], 2),
                    f'{i+1}th K value does not match')

            # R
            self.assertEqual(len(camera_info.r), len(received_camera_info.r))
            for i in range(len(received_camera_info.r)):
                self.assertEqual(
                    round(camera_info.r[i], 2), round(received_camera_info.r[i], 2),
                    f'{i+1}th R value does not match')

            # P
            self.assertEqual(len(camera_info.p), len(received_camera_info.p))
            # The P matrix is a 3*4 matrix, while left 3*3 matrix should be identical with K-matrix
            for i in range(3):
                for j in range(4):
                    # Compare with the translation vector
                    if(j == 3):
                        self.assertEqual(
                            round(camera_info.p[i*4+j], 2),
                            round(received_camera_info.p[i*4+j], 2),
                            f'{i+1}th P value does not match')
                    # Compare with the K matrix
                    else:
                        self.assertEqual(
                            round(camera_info.k[i*3+j], 2),
                            round(received_camera_info.p[i*4+j], 2),
                            f'{i+1}th P value does not match')

            print('The received camera info is verified successfully')
        finally:
            self.node.destroy_subscription(received_message_sub)
            self.node.destroy_publisher(camera_info_pub)
