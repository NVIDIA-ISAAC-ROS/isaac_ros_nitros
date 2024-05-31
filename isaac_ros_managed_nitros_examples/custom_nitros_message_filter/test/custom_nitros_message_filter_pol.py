# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import os
import pathlib
import time

from custom_nitros_message_filter_interfaces.msg import SyncStatus
from isaac_ros_test import IsaacROSBaseTest, JSONConversion
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import pytest
import rclpy
from sensor_msgs.msg import CameraInfo, Image


@pytest.mark.rostest
def generate_test_description():
    synchronizer_node = ComposableNode(
        name='synchronizer_node',
        package='custom_nitros_message_filter',
        plugin='custom_nitros_message_filter::SynchronizerNode',
        namespace=CustomNitrosMessageFilterPOLTest.generate_namespace(),
        )

    return CustomNitrosMessageFilterPOLTest.generate_test_description([
        ComposableNodeContainer(
            name='custom_nitros_message_filter_pol_test_container',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[synchronizer_node],
            namespace=CustomNitrosMessageFilterPOLTest.generate_namespace(),
            output='screen',
            arguments=['--ros-args', '--log-level', 'info']
        )
    ])


class CustomNitrosMessageFilterPOLTest(IsaacROSBaseTest):

    filepath = pathlib.Path(os.path.dirname(__file__))

    @IsaacROSBaseTest.for_each_test_case()
    def test_message_sync(self, test_folder):
        """
        Test image + camera info synchronization.

        Test that the custom NITROS image and standard camera info messages
        are properly synchronized by comparing sent messages to response array.
        """
        TIMEOUT = 5

        # Masks indicating which subset of messages to send in each subtest
        # To validate correct synchronization behavior, not all messages are sent
        # in each subtest. At least one message must be sent to trigger a response
        MESSAGE_MASKS = [
            (True, True),
            (True, False),
            (False, True),
            (True, True)
        ]

        # Timing offset for testing each mask without cross-talk
        TIMING_OFFSET = 1000

        received_messages = {}
        self.generate_namespace_lookup(['image', 'camera_info', 'status'])

        image_pub = self.node.create_publisher(
            Image, self.namespaces['image'], self.DEFAULT_QOS)
        camera_info_pub = self.node.create_publisher(
            CameraInfo, self.namespaces['camera_info'], self.DEFAULT_QOS)

        subs = self.create_logging_subscribers(
            [('status', SyncStatus)], received_messages, accept_multiple_messages=True)

        try:
            image = JSONConversion.load_image_from_json(
                test_folder / 'image.json')
            camera_info = JSONConversion.load_camera_info_from_json(
                test_folder / 'camera_info.json')

            # Conditionally send data to test synchronization
            # Data will be sent with continually increasing timestamps
            for i, (include_image, include_camera_info) in enumerate(MESSAGE_MASKS):

                # Wait at most TIMEOUT seconds for subscriber to respond
                end_time = time.time() + TIMEOUT
                done = False

                # Record the timing window reserved for this mask's subtest
                start_timestamp = i * TIMING_OFFSET
                end_timestamp = (i + 1) * TIMING_OFFSET
                current_timestamp = start_timestamp
                while time.time() < end_time:

                    # Apply desired image and camera_info timestamps
                    if include_image:
                        image.header.stamp.sec = current_timestamp
                        image.header.stamp.nanosec = 0
                        image_pub.publish(image)

                    if include_camera_info:
                        camera_info.header.stamp.sec = current_timestamp
                        camera_info.header.stamp.nanosec = 0
                        camera_info_pub.publish(camera_info)

                    rclpy.spin_once(self.node, timeout_sec=(0.1))
                    current_timestamp += 1

                    # Continue sending messages until response received
                    if 'status' not in received_messages or len(received_messages['status']) == 0:
                        continue

                    # Collect latest response and check if it belongs to the current subtest
                    status = received_messages['status'][-1]
                    if start_timestamp <= status.stamp.sec < end_timestamp:
                        done = True
                        break

                self.assertTrue(
                    done,
                    'Expected to receive a response for message mask: '
                    f'({include_image}, {include_camera_info})'
                )

                self.assertEqual(
                    status.messages_present[0],
                    include_image,
                    'Unexpected mismatch in presence of image!'
                )

                self.assertEqual(
                    status.messages_present[1],
                    include_camera_info,
                    'Unexpected mismatch in presence of camera info!'
                )

                if include_image and include_camera_info:
                    self.assertTrue(
                        status.exact_time_match,
                        'Received both messages but with different timestamps!'
                    )
                else:
                    self.assertFalse(
                        status.exact_time_match,
                        'Response claims that two timestamps matched, '
                        'but only one message was sent!'
                    )

        finally:
            self.node.destroy_publisher(image_pub)
            self.node.destroy_subscription(subs)
