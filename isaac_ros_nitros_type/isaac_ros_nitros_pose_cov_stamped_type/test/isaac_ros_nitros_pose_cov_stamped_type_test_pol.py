# Copyright (c) 2023, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

"""Proof-of-Life test for the NitrosPoseCovStamped type adapter."""

import os
import pathlib
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
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
    test_ns = IsaacROSNitrosPoseCovStampedTest.generate_namespace()
    container = ComposableNodeContainer(
        name='test_container',
        namespace='isaac_ros_nitros_container',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_nitros_pose_cov_stamped_type',
                plugin='nvidia::isaac_ros::nitros::NitrosPoseCovStampedForwardNode',
                name='NitrosPoseCovStampedForwardNode',
                namespace=test_ns,
                parameters=[{
                    'compatible_format': 'nitros_pose_cov_stamped'
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

    return IsaacROSNitrosPoseCovStampedTest.generate_test_description([
        container,
        launch.actions.TimerAction(
            period=2.5, actions=[launch_testing.actions.ReadyToTest()])
    ])


class IsaacROSNitrosPoseCovStampedTest(IsaacROSBaseTest):
    """Validate NitrosPoseCovStamped type adapter."""

    filepath = pathlib.Path(os.path.dirname(__file__))

    @IsaacROSBaseTest.for_each_test_case(subfolder='nitros_pose_cov_stamped')
    def test_nitros_pose_cov_stamped_type_conversions(self, test_folder) -> None:
        """Expect the tag from NitrosPoseCovStamped type to be compatible with source."""
        self.generate_namespace_lookup(['input', 'output'])
        received_messages = {}

        received_message_sub = self.create_logging_subscribers(
            subscription_requests=[('output', PoseWithCovarianceStamped)],
            received_messages=received_messages
        )

        pose_cov_stamped_pub = self.node.create_publisher(
            PoseWithCovarianceStamped, self.namespaces['input'], self.DEFAULT_QOS)

        try:
            print(test_folder)
            pose_cov_stamped = self.load_pose_cov_stamped_from_json(
                test_folder / 'pose_cov_stamped.json')

            # Wait at most TIMEOUT seconds for subscriber to respond
            TIMEOUT = 10
            end_time = time.time() + TIMEOUT

            done = False
            while time.time() < end_time:
                timestamp = self.node.get_clock().now().to_msg()
                pose_cov_stamped.header.stamp = timestamp

                pose_cov_stamped_pub.publish(pose_cov_stamped)

                rclpy.spin_once(self.node, timeout_sec=0.1)

                # If we have received a message on the output topic, break
                if 'output' in received_messages:
                    done = True
                    break

            self.assertTrue(done, "Didn't receive output on the output topic!")

            received_pose_cov_stamped = received_messages['output']

            self.assertEqual(pose_cov_stamped.pose.pose.position.x,
                             received_pose_cov_stamped.pose.pose.position.x)
            self.assertEqual(pose_cov_stamped.pose.pose.position.y,
                             received_pose_cov_stamped.pose.pose.position.y)
            self.assertEqual(pose_cov_stamped.pose.pose.position.z,
                             received_pose_cov_stamped.pose.pose.position.z)

            self.assertAlmostEqual(pose_cov_stamped.pose.pose.orientation.x,
                                   received_pose_cov_stamped.pose.pose.orientation.x, None,
                                   'did not receive expected orientation', 0.001)
            self.assertAlmostEqual(pose_cov_stamped.pose.pose.orientation.y,
                                   received_pose_cov_stamped.pose.pose.orientation.y, None,
                                   'did not receive expected orientation', 0.001)
            self.assertAlmostEqual(pose_cov_stamped.pose.pose.orientation.z,
                                   received_pose_cov_stamped.pose.pose.orientation.z, None,
                                   'did not receive expected orientation', 0.001)
            self.assertAlmostEqual(pose_cov_stamped.pose.pose.orientation.w,
                                   received_pose_cov_stamped.pose.pose.orientation.w, None,
                                   'did not receive expected orientation', 0.001)
            for i in range(36):
                self.assertEqual(pose_cov_stamped.pose.covariance[i],
                                 received_pose_cov_stamped.pose.covariance[i])
            print('The received pose cov stamped message has been verified successfully')
        finally:
            self.node.destroy_subscription(received_message_sub)
            self.node.destroy_publisher(pose_cov_stamped_pub)

    @staticmethod
    def load_pose_cov_stamped_from_json(
            json_filepath: pathlib.Path) -> PoseWithCovarianceStamped:
        """
        Load a PoseWithCovarianceStamped message from a JSON filepath.

        Parameters
        ----------
        json_filepath : Path
            The path to a JSON file containing the PoseWithCovarianceStamped fields

        Returns
        -------
        PoseWithCovarianceStamped
            Generated PoseWithCovarianceStamped message

        """
        pose_cov_stamped_json = JSONConversion.load_from_json(
            json_filepath)

        pose_cov_stamped = PoseWithCovarianceStamped()
        pose_cov_stamped.header.frame_id = pose_cov_stamped_json[
            'header']['frame_id']
        pose_cov_stamped.header.stamp.sec = pose_cov_stamped_json[
            'header']['stamp']['sec']
        pose_cov_stamped.header.stamp.nanosec = pose_cov_stamped_json[
            'header']['stamp']['nanosec']
        pose_cov_stamped.pose.pose.position.x = pose_cov_stamped_json[
            'pose']['pose']['position']['x']
        pose_cov_stamped.pose.pose.position.y = pose_cov_stamped_json[
            'pose']['pose']['position']['y']
        pose_cov_stamped.pose.pose.position.z = pose_cov_stamped_json[
            'pose']['pose']['position']['z']
        pose_cov_stamped.pose.pose.orientation.x = pose_cov_stamped_json[
            'pose']['pose']['orientation']['x']
        pose_cov_stamped.pose.pose.orientation.y = pose_cov_stamped_json[
            'pose']['pose']['orientation']['y']
        pose_cov_stamped.pose.pose.orientation.z = pose_cov_stamped_json[
            'pose']['pose']['orientation']['z']
        pose_cov_stamped.pose.pose.orientation.w = pose_cov_stamped_json[
            'pose']['pose']['orientation']['w']
        for i in range(36):
            pose_cov_stamped.pose.covariance[i] = pose_cov_stamped_json['pose']['covariance'][i]
        return pose_cov_stamped
