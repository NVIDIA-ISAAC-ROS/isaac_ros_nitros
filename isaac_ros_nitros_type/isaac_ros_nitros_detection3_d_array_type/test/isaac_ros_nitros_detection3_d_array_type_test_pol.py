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

"""Proof-of-Life test for the NitrosDetection3DArray type adapter."""

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
from vision_msgs.msg import Detection3D, Detection3DArray, \
    ObjectHypothesis, ObjectHypothesisWithPose


@pytest.mark.rostest
def generate_test_description():
    """Generate launch description with all ROS 2 nodes for testing."""
    test_ns = IsaacROSNitrosDetection3DArrayTest.generate_namespace()
    container = ComposableNodeContainer(
        name='test_container',
        namespace='isaac_ros_nitros_container',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_nitros_detection3_d_array_type',
                plugin='nvidia::isaac_ros::nitros::NitrosDetection3DArrayForwardNode',
                name='NitrosDetection3DArrayForwardNode',
                namespace=test_ns,
                parameters=[{
                    'compatible_format': 'nitros_detection3_d_array'
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

    return IsaacROSNitrosDetection3DArrayTest.generate_test_description([
        container,
        launch.actions.TimerAction(
            period=2.5, actions=[launch_testing.actions.ReadyToTest()])
    ])


class IsaacROSNitrosDetection3DArrayTest(IsaacROSBaseTest):
    """Validate NitrosDetection3DArray type adapter."""

    filepath = pathlib.Path(os.path.dirname(__file__))

    @IsaacROSBaseTest.for_each_test_case(subfolder='nitros_detection3_d_array')
    def test_nitros_detection3_d_array_type_conversions(self, test_folder) -> None:
        """Expect the tag from NitrosDetection3DArray type to be compatible with source."""
        self.generate_namespace_lookup(['input', 'output'])
        received_messages = {}

        received_message_sub = self.create_logging_subscribers(
            subscription_requests=[('output', Detection3DArray)],
            received_messages=received_messages
        )

        nitros_detection3_d_array_pub = self.node.create_publisher(
            Detection3DArray, self.namespaces['input'], self.DEFAULT_QOS)

        try:
            print(test_folder)
            nitros_detection3_d_array = self.load_nitros_detection3_d_array_from_json(
                test_folder / 'nitros_detection3_d_array.json')

            # Wait at most TIMEOUT seconds for subscriber to respond
            TIMEOUT = 10
            end_time = time.time() + TIMEOUT

            done = False
            while time.time() < end_time:
                timestamp = self.node.get_clock().now().to_msg()
                nitros_detection3_d_array.header.stamp = timestamp

                nitros_detection3_d_array_pub.publish(
                    nitros_detection3_d_array)

                rclpy.spin_once(self.node, timeout_sec=0.1)

                # If we have received a message on the output topic, break
                if 'output' in received_messages:
                    done = True
                    break

            self.assertTrue(done, "Didn't receive output on the output topic!")

            received_nitros_detection3_d_array = received_messages['output']
            # Header
            # Frame ID is to be passed from NitrosPoseArray to ROS message
            # self.assertEqual(nitros_detection3_d_array.header.frame_id,
            #                 received_nitros_detection3_d_array.header.frame_id)

            self.assertEqual(len(nitros_detection3_d_array.detections),
                             len(received_nitros_detection3_d_array.detections))

            if(len(nitros_detection3_d_array.detections) ==
                    len(received_nitros_detection3_d_array.detections)):
                for detection_count, detection3_d in \
                        enumerate(nitros_detection3_d_array.detections):
                    received_detection3_d = received_nitros_detection3_d_array.detections[
                        detection_count]
                    self.assertEqual(len(detection3_d.results),
                                     len(received_detection3_d.results))
                    if(len(detection3_d.results) ==
                            len(received_detection3_d.results)):
                        for result_count, result in enumerate(detection3_d.results):
                            received_result = received_detection3_d.results[result_count]
                            self.assertEqual(result.hypothesis.class_id,
                                             received_result.hypothesis.class_id)
                            self.assertAlmostEqual(result.hypothesis.score,
                                                   received_result.hypothesis.score, None,
                                                   'hypothesis score is not accurate', 0.05)
                    self.assertEqual(detection3_d.bbox.center.position.x,
                                     received_detection3_d.bbox.center.position.x)
                    self.assertEqual(detection3_d.bbox.center.position.y,
                                     received_detection3_d.bbox.center.position.y)
                    self.assertEqual(detection3_d.bbox.center.position.z,
                                     received_detection3_d.bbox.center.position.z)
                    self.assertEqual(detection3_d.bbox.center.orientation.x,
                                     received_detection3_d.bbox.center.orientation.x)
                    self.assertEqual(detection3_d.bbox.center.orientation.y,
                                     received_detection3_d.bbox.center.orientation.y)
                    self.assertEqual(detection3_d.bbox.center.orientation.z,
                                     received_detection3_d.bbox.center.orientation.z)
                    self.assertEqual(detection3_d.bbox.center.orientation.w,
                                     received_detection3_d.bbox.center.orientation.w)
                    self.assertEqual(detection3_d.bbox.size.x,
                                     received_detection3_d.bbox.size.x)
                    self.assertEqual(detection3_d.bbox.size.y,
                                     received_detection3_d.bbox.size.y)
                    self.assertEqual(detection3_d.bbox.size.z,
                                     received_detection3_d.bbox.size.z)
            print('The received detection 3D array has been verified successfully')
        finally:
            self.node.destroy_subscription(received_message_sub)
            self.node.destroy_publisher(nitros_detection3_d_array_pub)

    @staticmethod
    def load_nitros_detection3_d_array_from_json(
            json_filepath: pathlib.Path) -> Detection3DArray:
        """
        Load a Detection3DArray message from a JSON filepath.

        Parameters
        ----------
        json_filepath : Path
            The path to a JSON file containing the Detection3DArray fields

        Returns
        -------
        Detection3DArray
            Generated Detection3DArray message

        """
        detection3_d_array_json = JSONConversion.load_from_json(
            json_filepath)

        detection3_d_array = Detection3DArray()
        detection3_d_array.header.frame_id = detection3_d_array_json[
            'header']['frame_id']
        for detection in detection3_d_array_json['detections']:
            detection3_d = Detection3D()
            for result in detection['results']:
                object_hypothesis_with_pose = ObjectHypothesisWithPose()
                object_hypothesis = ObjectHypothesis()
                object_hypothesis.class_id = result['hypothesis']['class_id']
                object_hypothesis.score = result['hypothesis']['score']
                object_hypothesis_with_pose.hypothesis = object_hypothesis
                detection3_d.results.append(object_hypothesis_with_pose)
            detection3_d.bbox.center.position.x = detection['bbox']['center']['position']['x']
            detection3_d.bbox.center.position.y = detection['bbox']['center']['position']['y']
            detection3_d.bbox.center.position.z = detection['bbox']['center']['position']['z']

            orientation_dict = detection['bbox']['center']['orientation']
            detection3_d.bbox.center.orientation.x = orientation_dict['x']
            detection3_d.bbox.center.orientation.y = orientation_dict['y']
            detection3_d.bbox.center.orientation.z = orientation_dict['z']
            detection3_d.bbox.center.orientation.w = orientation_dict['w']

            detection3_d.bbox.size.x = detection['bbox']['size_x']
            detection3_d.bbox.size.y = detection['bbox']['size_y']
            detection3_d.bbox.size.z = detection['bbox']['size_z']

            detection3_d_array.detections.append(detection3_d)
        return detection3_d_array
