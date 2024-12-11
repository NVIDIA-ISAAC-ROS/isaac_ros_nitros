# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2022-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import time

from diagnostic_msgs.msg import DiagnosticArray
from isaac_ros_test import IsaacROSBaseTest
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import launch_testing
import pytest
import rclpy
from std_msgs.msg import Empty


@pytest.mark.rostest
def generate_test_description():
    """Generate launch description with NitrosEmptyForwardNode test node."""
    test_ns = IsaacROSNitrosNodeTest.generate_namespace()
    container = ComposableNodeContainer(
        name='image_container',
        namespace='isaac_ros_nitros_container',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_nitros',
                plugin='nvidia::isaac_ros::nitros::NitrosEmptyForwardNode',
                name='isaac_ros_nitros',
                namespace=test_ns,
                parameters=[{
                        'compatible_format': 'nitros_empty',
                        'enable_all_diagnostics': True,
                        'diagnostics_publish_rate': 5.0,
                        'filter_window_size': 10,
                        'topics_list': ['topic_forward_output'],
                        'jitter_tolerance_us': 100,
                        'expected_fps_list': [50.0]
                }]
            ),
            ComposableNode(
                package='isaac_ros_nitros',
                plugin='nvidia::isaac_ros::nitros::NitrosEmptyForwardNode',
                name='isaac_ros_nitros',
                namespace=test_ns + '/mid1',
                parameters=[{
                        'compatible_format': 'nitros_empty',
                        'enable_all_diagnostics': False,
                }],
                remappings=[
                    (test_ns + '/mid1/topic_forward_input',
                     test_ns + '/topic_forward_output'),
                    (test_ns + '/mid1/topic_forward_output',
                     test_ns + '/final/topic_forward_output'),
                ]
            )
        ],
        output='both',
        arguments=['--ros-args', '--log-level', 'info'],
    )

    return IsaacROSNitrosNodeTest.generate_test_description([
        container,
        launch.actions.TimerAction(
            period=2.5, actions=[launch_testing.actions.ReadyToTest()])
    ])


class IsaacROSNitrosNodeTest(IsaacROSBaseTest):
    """
    Proof-of-Life Test for Isaac ROS Nitros Node.

    1. Sets up ROS publisher to send Empty values
    2. Sets up ROS subscriber to listen to output channel of NitrosNode
    3. Verify received messages
    """

    def test_forward_node(self) -> None:
        self.node._logger.info('Starting Isaac ROS NitrosNode POL Test')

        # Subscriber
        received_messages = {}

        test_subscribers = [
            ('/diagnostics', DiagnosticArray)
        ]

        subs = self.create_logging_subscribers(
            subscription_requests=test_subscribers,
            received_messages=received_messages,
            use_namespace_lookup=False,
            accept_multiple_messages=True
        )

        # Publisher
        publisher_topic_namespace = self.generate_namespace('topic_forward_input')
        pub = self.node.create_publisher(
            Empty,
            publisher_topic_namespace,
            self.DEFAULT_QOS)

        try:
            # Construct test message
            msg = Empty()

            # Testing frame rate
            test_frame_rate = 50

            # Start sending messages
            self.node.get_logger().info('Start publishing messages')

            # Publish messages at a fixed frame rate set by test_frame_rate
            start_time = time.time()
            end_time = time.time() + 2
            sent_count = 0
            while time.time() < end_time:
                sent_count += 1
                pub.publish(msg)
                # Run the queued ROS subscriber callbacks
                rclpy.spin_once(self.node, timeout_sec=0)
                time.sleep(1 / test_frame_rate)
                if sent_count == 10:
                    # add a dropped frame be delaying here
                    time.sleep(.5)

            measured_frame_rate = sent_count / (time.time() - start_time)
            self.node.get_logger().info(f'sent_count: {sent_count}')
            self.node.get_logger().info(f'time: {time.time() - start_time}')
            self.node.get_logger().info(f'frame rate: {measured_frame_rate}')

            # Assert that we have received atleast one message on the output topic
            self.assertNotEqual(
                len(received_messages['/diagnostics']),
                0)

            if len(received_messages['/diagnostics']) > 0:
                # Assert that the measured frame rate is within the acceptable error window
                print('message len: ', len(received_messages['/diagnostics']))
                values_received = received_messages['/diagnostics'][-1].status[0].values
                diagnostics_values = {
                    'frame_rate_node': 0,
                    'num_non_increasing_msg': 0,
                    'num_jitter_outliers_msg': 0,
                    'num_jitter_outliers_node': 0,
                    'max_abs_jitter_msg': 0,
                    'max_abs_jitter_node': 0,
                    'mean_abs_jitter_msg': 0,
                    'mean_abs_jitter_node': 0,
                    'frame_rate_msg': 0,
                    'total_dropped_frames': 0
                }

                for kv in values_received:
                    if kv.key in diagnostics_values:
                        diagnostics_values[kv.key] = float(kv.value)

                # We are not publishing that consitently above, and this test may be running
                # on a busy machine so we allow a large error window
                self.assertAlmostEqual(diagnostics_values['frame_rate_node'], test_frame_rate,
                                       None, 'measured recv rate error is too high', 5.0)
                # The forward node used for testing always publishes a 0 msg timestamp
                self.assertEqual(diagnostics_values['frame_rate_msg'], 0)

                self.assertEqual(diagnostics_values['total_dropped_frames'], 0)

                # Additional assertions for other diagnostic values
                self.assertEqual(diagnostics_values['num_non_increasing_msg'], 0)

                self.assertEqual(diagnostics_values['num_jitter_outliers_msg'], 0)

                # we added one delayed frame, so we should have at least one jitter outlier
                # we have have more if the system was very busy
                self.assertGreaterEqual(diagnostics_values['num_jitter_outliers_node'], 1)

                self.assertGreaterEqual(diagnostics_values['max_abs_jitter_msg'], 0)
                self.assertGreaterEqual(diagnostics_values['max_abs_jitter_node'], 0)
                self.assertGreaterEqual(diagnostics_values['mean_abs_jitter_msg'], 0)
                self.assertGreaterEqual(diagnostics_values['mean_abs_jitter_node'], 0)

        finally:
            [self.node.destroy_subscription(sub) for sub in subs]
            self.assertTrue(self.node.destroy_publisher(pub))
