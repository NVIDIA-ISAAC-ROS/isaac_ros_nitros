# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

"""Proof-of-Life test for the NitrosDetection2DArray type adapter."""

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
from vision_msgs.msg import Detection2D, Detection2DArray, \
    ObjectHypothesis, ObjectHypothesisWithPose


@pytest.mark.rostest
def generate_test_description():
    """Generate launch description with all ROS 2 nodes for testing."""
    test_ns = IsaacROSNitrosDetection2DArrayTest.generate_namespace()
    container = ComposableNodeContainer(
        name='test_container',
        namespace='isaac_ros_nitros_container',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_nitros_detection2_d_array_type',
                plugin='nvidia::isaac_ros::nitros::NitrosDetection2DArrayForwardNode',
                name='NitrosDetection2DArrayForwardNode',
                namespace=test_ns,
                parameters=[{
                    'compatible_format': 'nitros_detection2_d_array'
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

    return IsaacROSNitrosDetection2DArrayTest.generate_test_description([
        container,
        launch.actions.TimerAction(
            period=2.5, actions=[launch_testing.actions.ReadyToTest()])
    ])


class IsaacROSNitrosDetection2DArrayTest(IsaacROSBaseTest):
    """Validate NitrosDetection2DArray type adapter."""

    filepath = pathlib.Path(os.path.dirname(__file__))

    @IsaacROSBaseTest.for_each_test_case(subfolder='nitros_detection2_d_array')
    def test_nitros_detection2_d_array_type_conversions(self, test_folder) -> None:
        """Expect the tag from NitrosDetection2DArray type to be compatible with source."""
        self.generate_namespace_lookup(['input', 'output'])
        received_messages = {}

        received_message_sub = self.create_logging_subscribers(
            subscription_requests=[('output', Detection2DArray)],
            received_messages=received_messages
        )

        nitros_detection2_d_array_pub = self.node.create_publisher(
            Detection2DArray, self.namespaces['input'], self.DEFAULT_QOS)

        try:
            print(test_folder)
            nitros_detection2_d_array = self.load_nitros_detection2_d_array_from_json(
                test_folder / 'nitros_detection2_d_array.json')

            # Wait at most TIMEOUT seconds for subscriber to respond
            TIMEOUT = 10
            end_time = time.time() + TIMEOUT

            done = False
            while time.time() < end_time:
                timestamp = self.node.get_clock().now().to_msg()
                nitros_detection2_d_array.header.stamp = timestamp

                nitros_detection2_d_array_pub.publish(
                    nitros_detection2_d_array)

                rclpy.spin_once(self.node, timeout_sec=0.1)

                # If we have received a message on the output topic, break
                if 'output' in received_messages:
                    done = True
                    break

            self.assertTrue(done, "Didn't receive output on the output topic!")

            received_nitros_detection2_d_array = received_messages['output']

            self.assertEqual(len(nitros_detection2_d_array.detections),
                             len(received_nitros_detection2_d_array.detections))

            if(len(nitros_detection2_d_array.detections) ==
                    len(received_nitros_detection2_d_array.detections)):
                for detection_count, detection2_d in \
                        enumerate(nitros_detection2_d_array.detections):
                    received_detection2_d = received_nitros_detection2_d_array.detections[
                        detection_count]
                    self.assertEqual(len(detection2_d.results),
                                     len(received_detection2_d.results))
                    if(len(detection2_d.results) ==
                            len(received_detection2_d.results)):
                        for result_count, result in enumerate(detection2_d.results):
                            received_result = received_detection2_d.results[result_count]
                            self.assertEqual(result.hypothesis.class_id,
                                             received_result.hypothesis.class_id)
                            self.assertAlmostEqual(result.hypothesis.score,
                                                   received_result.hypothesis.score, None,
                                                   'hypothesis score is not accurate', 0.05)
                    self.assertEqual(detection2_d.bbox.center.position.x,
                                     received_detection2_d.bbox.center.position.x)
                    self.assertEqual(detection2_d.bbox.center.position.y,
                                     received_detection2_d.bbox.center.position.y)
                    self.assertEqual(detection2_d.bbox.size_x,
                                     received_detection2_d.bbox.size_x)
                    self.assertEqual(detection2_d.bbox.size_y,
                                     received_detection2_d.bbox.size_y)
            print('The received detection 2D array has been verified successfully')
        finally:
            self.node.destroy_subscription(received_message_sub)
            self.node.destroy_publisher(nitros_detection2_d_array_pub)

    @staticmethod
    def load_nitros_detection2_d_array_from_json(
            json_filepath: pathlib.Path) -> Detection2DArray:
        """
        Load a Detection2DArray message from a JSON filepath.

        Parameters
        ----------
        json_filepath : Path
            The path to a JSON file containing the Detection2DArray fields

        Returns
        -------
        Detection2DArray
            Generated Detection2DArray message

        """
        detection2_d_array_json = JSONConversion.load_from_json(
            json_filepath)

        detection2_d_array = Detection2DArray()
        detection2_d_array.header.frame_id = detection2_d_array_json[
            'header']['frame_id']
        for detection in detection2_d_array_json['detections']:
            detection2_d = Detection2D()
            for result in detection['results']:
                object_hypothesis_with_pose = ObjectHypothesisWithPose()
                object_hypothesis = ObjectHypothesis()
                object_hypothesis.class_id = result['hypothesis']['class_id']
                object_hypothesis.score = result['hypothesis']['score']
                object_hypothesis_with_pose.hypothesis = object_hypothesis
                detection2_d.results.append(object_hypothesis_with_pose)
            detection2_d.bbox.center.position.x = detection['bbox']['center']['position']['x']
            detection2_d.bbox.center.position.y = detection['bbox']['center']['position']['y']
            detection2_d.bbox.size_x = detection['bbox']['size_x']
            detection2_d.bbox.size_y = detection['bbox']['size_y']
            detection2_d_array.detections.append(detection2_d)
        return detection2_d_array
