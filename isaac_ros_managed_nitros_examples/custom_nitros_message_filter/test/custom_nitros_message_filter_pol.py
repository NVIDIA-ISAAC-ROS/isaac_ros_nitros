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

    @staticmethod
    def _format_sync_status(status: SyncStatus) -> str:
        # Keep failure output compact but actionable.
        try:
            stamp_sec = status.stamp.sec
        except Exception:
            stamp_sec = '<unavailable>'
        try:
            present = list(status.messages_present)
        except Exception:
            present = '<unavailable>'
        try:
            exact = status.exact_time_match
        except Exception:
            exact = '<unavailable>'
        return f'stamp.sec={stamp_sec}, messages_present={present}, exact_time_match={exact}'

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
            # Ensure synchronizer subscriptions are connected before publishing.
            end_time = time.time() + TIMEOUT
            while time.time() < end_time:
                image_ready = image_pub.get_subscription_count() > 0
                camera_info_ready = camera_info_pub.get_subscription_count() > 0
                if image_ready and camera_info_ready:
                    break
                rclpy.spin_once(self.node, timeout_sec=0.1)

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
                matched_status = None
                status_index = len(received_messages['status'])
                observed_status_total = 0
                observed_in_window = 0
                observed_out_of_window = 0
                rejected_too_short = 0
                rejected_image_presence = 0
                rejected_camera_info_presence = 0
                rejected_exact_time = 0
                last_in_window_status = None
                closest_out_of_window_status = None
                closest_out_of_window_distance = None

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

                    # Check any new status messages for a match to this subtest.
                    while status_index < len(received_messages['status']):
                        status = received_messages['status'][status_index]
                        status_index += 1
                        observed_status_total += 1

                        if not (start_timestamp <= status.stamp.sec < end_timestamp):
                            observed_out_of_window += 1
                            # Track the closest stamp we saw outside the window to help debugging.
                            # (Useful when offsets drift or another subtest's
                            # messages are leaking in.)
                            stamp_sec = status.stamp.sec
                            if stamp_sec < start_timestamp:
                                dist = start_timestamp - stamp_sec
                            else:
                                dist = stamp_sec - (end_timestamp - 1)
                            if (
                                (closest_out_of_window_distance is None)
                                or (dist < closest_out_of_window_distance)
                            ):
                                closest_out_of_window_distance = dist
                                closest_out_of_window_status = status
                            continue

                        observed_in_window += 1
                        last_in_window_status = status

                        if len(status.messages_present) < 2:
                            rejected_too_short += 1
                            continue

                        if status.messages_present[0] != include_image:
                            rejected_image_presence += 1
                            continue

                        if status.messages_present[1] != include_camera_info:
                            rejected_camera_info_presence += 1
                            continue

                        if status.exact_time_match != (include_image and include_camera_info):
                            rejected_exact_time += 1
                            continue

                        matched_status = status
                        done = True
                        break

                    if done:
                        break

                if not done:
                    expected_exact_time_match = include_image and include_camera_info
                    failure_details = [
                        f'Did not find matching SyncStatus for message mask '
                        f'({include_image}, {include_camera_info}) within {TIMEOUT}s.',
                        f'Expected: messages_present=[{include_image}, {include_camera_info}], '
                        f'exact_time_match={expected_exact_time_match}.',
                        f'Subtest time window (stamp.sec): [{start_timestamp}, {end_timestamp}).',
                        f'New status msgs observed: total={observed_status_total}, '
                        f'in_window={observed_in_window}, out_of_window={observed_out_of_window}.',
                    ]

                    if observed_status_total == 0:
                        failure_details.append(
                            'Failure case: no new `status` messages arrived for this subtest.'
                        )
                    elif observed_in_window == 0:
                        failure_details.append(
                            'Failure case: received `status` messages, but none fell within the '
                            'reserved timestamp window for this subtest.'
                        )
                        if closest_out_of_window_status is not None:
                            failure_details.append(
                                'Closest out-of-window SyncStatus observed: '
                                f'{self._format_sync_status(closest_out_of_window_status)}'
                            )
                    else:
                        failure_details.append(
                            'Rejections within window: '
                            f'too_short_messages_present={rejected_too_short}, '
                            f'image_presence_mismatch={rejected_image_presence}, '
                            f'camera_info_presence_mismatch={rejected_camera_info_presence}, '
                            f'exact_time_match_mismatch={rejected_exact_time}.'
                        )
                        if last_in_window_status is not None:
                            failure_details.append(
                                'Last in-window SyncStatus observed: '
                                f'{self._format_sync_status(last_in_window_status)}'
                            )

                    self.fail('\n'.join(failure_details))

                status = matched_status
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
