# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

"""Proof-of-Life test for the NitrosPoseArray type adapter."""

import os
import pathlib
import time

from geometry_msgs.msg import PoseArray

from isaac_ros_test import IsaacROSBaseTest, JSONConversion

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import pytest
import rclpy


@pytest.mark.rostest
def generate_test_description():
    """Generate launch description with all ROS 2 nodes for testing."""
    test_ns = IsaacROSNitrosPoseArrayTest.generate_namespace()
    container = ComposableNodeContainer(
        name='test_container',
        namespace='isaac_ros_nitros_container',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_nitros_pose_array_type',
                plugin='nvidia::isaac_ros::nitros::NitrosPoseArrayForwardNode',
                name='NitrosPoseArrayForwardNode',
                namespace=test_ns,
                parameters=[{
                    'compatible_format': 'nitros_pose_array'
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

    return IsaacROSNitrosPoseArrayTest.generate_test_description(
        [container],
        node_startup_delay=2.5
    )


class IsaacROSNitrosPoseArrayTest(IsaacROSBaseTest):
    """Validate NitrosPoseArray type adapter."""

    filepath = pathlib.Path(os.path.dirname(__file__))

    @IsaacROSBaseTest.for_each_test_case()
    def test_nitros_pose_array_type_conversions(self, test_folder) -> None:
        """Expect the pose array from NitrosPoseArray type to be compatible with source."""
        self.generate_namespace_lookup(['input', 'output'])
        received_messages = {}

        received_message_sub = self.create_logging_subscribers(
            subscription_requests=[('output', PoseArray)],
            received_messages=received_messages
        )

        pose_array_pub = self.node.create_publisher(
            PoseArray, self.namespaces['input'], self.DEFAULT_QOS)

        try:
            pose_array = JSONConversion.load_pose_array_from_json(
                test_folder / 'pose_array.json')

            # Wait at most TIMEOUT seconds for subscriber to respond
            TIMEOUT = 2
            end_time = time.time() + TIMEOUT

            done = False
            while time.time() < end_time:
                timestamp = self.node.get_clock().now().to_msg()
                pose_array.header.stamp = timestamp

                pose_array_pub.publish(pose_array)

                rclpy.spin_once(self.node, timeout_sec=0.1)

                # If we have received a message on the output topic, break
                if 'output' in received_messages:
                    done = True
                    break

            self.assertTrue(done, "Didn't receive output on the output topic!")

            received_pose_array = received_messages['output']

            # Header
            # Frame ID is to be passed from NitrosPoseArray to ROS message
            # self.assertEqual(pose_array.header.frame_id, received_pose_array.header.frame_id)

            # Poses
            self.assertEqual(len(pose_array.poses), len(
                received_pose_array.poses), 'Number of poses does not match')

            for pose, received_pose in zip(pose_array.poses, received_pose_array.poses):
                self.assertEqual(
                    round(pose.position.x, 2), round(received_pose.position.x, 2),
                    'Position x value does not match')
                self.assertEqual(
                    round(pose.position.y, 2), round(received_pose.position.y, 2),
                    'Position y value does not match')
                self.assertEqual(
                    round(pose.position.z, 2), round(received_pose.position.z, 2),
                    'Position z value does not match')

                self.assertEqual(
                    round(pose.orientation.x, 2), round(received_pose.orientation.x, 2),
                    'Orientation x value does not match')
                self.assertEqual(
                    round(pose.orientation.y, 2), round(received_pose.orientation.y, 2),
                    'Orientation x value does not match')
                self.assertEqual(
                    round(pose.orientation.z, 2), round(received_pose.orientation.z, 2),
                    'Orientation x value does not match')
                self.assertEqual(
                    round(pose.orientation.w, 2), round(received_pose.orientation.w, 2),
                    'Orientation w value does not match')

            print('The received pose array is verified successfully')
        finally:
            self.node.destroy_subscription(received_message_sub)
            self.node.destroy_publisher(pose_array_pub)
