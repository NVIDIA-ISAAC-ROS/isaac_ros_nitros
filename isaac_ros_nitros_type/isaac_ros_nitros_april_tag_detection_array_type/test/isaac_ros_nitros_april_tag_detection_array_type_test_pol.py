# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

"""Proof-of-Life test for the NitrosAprilTagDetectionArray type adapter."""

import os
import pathlib
import time

from isaac_ros_apriltag_interfaces.msg import AprilTagDetection, AprilTagDetectionArray
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
    test_ns = IsaacROSNitrosAprilTagDetectionArrayTest.generate_namespace()
    container = ComposableNodeContainer(
        name='test_container',
        namespace='isaac_ros_nitros_container',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_nitros_april_tag_detection_array_type',
                plugin='nvidia::isaac_ros::nitros::NitrosAprilTagDetectionArrayForwardNode',
                name='NitrosAprilTagDetectionArrayForwardNode',
                namespace=test_ns,
                parameters=[{
                    'compatible_format': 'nitros_april_tag_detection_array'
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

    return IsaacROSNitrosAprilTagDetectionArrayTest.generate_test_description([
        container,
        launch.actions.TimerAction(
            period=2.5, actions=[launch_testing.actions.ReadyToTest()])
    ])


class IsaacROSNitrosAprilTagDetectionArrayTest(IsaacROSBaseTest):
    """Validate NitrosAprilTagDetectionArray type adapter."""

    filepath = pathlib.Path(os.path.dirname(__file__))

    @IsaacROSBaseTest.for_each_test_case(subfolder='nitros_april_tag_detection_array')
    def test_nitros_april_tag_detection_array_type_conversions(self, test_folder) -> None:
        """Expect the tag from NitrosAprilTagDetectionArray type to be compatible with source."""
        self.generate_namespace_lookup(['input', 'output'])
        received_messages = {}

        received_message_sub = self.create_logging_subscribers(
            subscription_requests=[('output', AprilTagDetectionArray)],
            received_messages=received_messages
        )

        april_tag_detection_array_pub = self.node.create_publisher(
            AprilTagDetectionArray, self.namespaces['input'], self.DEFAULT_QOS)

        try:
            print(test_folder)
            april_tag_detection_array = self.load_april_tag_detection_array_from_json(
                test_folder / 'april_tag_detection_array.json')

            # Wait at most TIMEOUT seconds for subscriber to respond
            TIMEOUT = 10
            end_time = time.time() + TIMEOUT

            done = False
            while time.time() < end_time:
                timestamp = self.node.get_clock().now().to_msg()
                april_tag_detection_array.header.stamp = timestamp

                april_tag_detection_array_pub.publish(april_tag_detection_array)

                rclpy.spin_once(self.node, timeout_sec=0.1)

                # If we have received a message on the output topic, break
                if 'output' in received_messages:
                    done = True
                    break

            self.assertTrue(done, "Didn't receive output on the output topic!")

            received_april_tag_detection_array = received_messages['output']

            self.assertEqual(len(april_tag_detection_array.detections),
                             len(received_april_tag_detection_array.detections))

            if(len(april_tag_detection_array.detections) ==
                    len(received_april_tag_detection_array.detections)):
                for count, april_tag_detection in enumerate(april_tag_detection_array.detections):
                    received_april_tag_detection = received_april_tag_detection_array.detections[
                        count]
                    self.assertEqual(april_tag_detection.family,
                                     received_april_tag_detection.family)
                    self.assertEqual(april_tag_detection.id,
                                     received_april_tag_detection.id)
                    for i in range(4):
                        self.assertEqual(april_tag_detection.corners[i].x,
                                         received_april_tag_detection.corners[i].x)
                        self.assertEqual(april_tag_detection.corners[i].y,
                                         received_april_tag_detection.corners[i].y)
                    self.assertEqual(april_tag_detection.pose.pose.pose.position.x,
                                     received_april_tag_detection.pose.pose.pose.position.x)
                    self.assertEqual(april_tag_detection.pose.pose.pose.position.y,
                                     received_april_tag_detection.pose.pose.pose.position.y)
                    self.assertEqual(april_tag_detection.pose.pose.pose.position.z,
                                     received_april_tag_detection.pose.pose.pose.position.z)
                    self.assertEqual(april_tag_detection.pose.pose.pose.orientation.x,
                                     received_april_tag_detection.pose.pose.pose.orientation.x)
                    self.assertEqual(april_tag_detection.pose.pose.pose.orientation.y,
                                     received_april_tag_detection.pose.pose.pose.orientation.y)
                    self.assertEqual(april_tag_detection.pose.pose.pose.orientation.z,
                                     received_april_tag_detection.pose.pose.pose.orientation.z)
                    self.assertEqual(april_tag_detection.pose.pose.pose.orientation.w,
                                     received_april_tag_detection.pose.pose.pose.orientation.w)
            print('The received april tag detection array has been verified successfully')
        finally:
            self.node.destroy_subscription(received_message_sub)
            self.node.destroy_publisher(april_tag_detection_array_pub)

    @staticmethod
    def load_april_tag_detection_array_from_json(
            json_filepath: pathlib.Path) -> AprilTagDetectionArray:
        """
        Load a AprilTagDetectionArray message from a JSON filepath.

        Parameters
        ----------
        json_filepath : Path
            The path to a JSON file containing the AprilTagDetectionArray fields

        Returns
        -------
        AprilTagDetectionArray
            Generated AprilTagDetectionArray message

        """
        apriltag_detection_array_json = JSONConversion.load_from_json(
            json_filepath)

        apriltag_detection_array = AprilTagDetectionArray()
        apriltag_detection_array.header.frame_id = apriltag_detection_array_json[
            'header']['frame_id']
        for detection in apriltag_detection_array_json['detections']:
            apriltag_detection = AprilTagDetection()
            apriltag_detection.id = detection['id']
            apriltag_detection.family = detection['family']
            apriltag_detection.center.x = detection['center']['x']
            apriltag_detection.center.y = detection['center']['y']
            for corner_index, corner in enumerate(detection['corners']):
                apriltag_detection.corners[corner_index].x = corner['x']
                apriltag_detection.corners[corner_index].y = corner['y']
            apriltag_detection.pose.header.frame_id = detection['pose']['header']['frame_id']
            apriltag_detection.pose.pose.pose.position.x = detection[
                'pose']['pose']['pose']['position']['x']
            apriltag_detection.pose.pose.pose.position.y = detection[
                'pose']['pose']['pose']['position']['y']
            apriltag_detection.pose.pose.pose.position.z = detection[
                'pose']['pose']['pose']['position']['z']
            apriltag_detection.pose.pose.pose.orientation.x = detection[
                'pose']['pose']['pose']['orientation']['x']
            apriltag_detection.pose.pose.pose.orientation.y = detection[
                'pose']['pose']['pose']['orientation']['y']
            apriltag_detection.pose.pose.pose.orientation.z = detection[
                'pose']['pose']['pose']['orientation']['z']
            apriltag_detection.pose.pose.pose.orientation.w = detection[
                'pose']['pose']['pose']['orientation']['w']
            apriltag_detection_array.detections.append(apriltag_detection)

        return apriltag_detection_array
