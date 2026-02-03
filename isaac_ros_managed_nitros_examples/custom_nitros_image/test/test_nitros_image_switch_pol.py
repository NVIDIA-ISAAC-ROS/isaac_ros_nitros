# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from cv_bridge import CvBridge
from isaac_ros_test import IsaacROSBaseTest
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import numpy as np

import pytest
import rclpy

from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import SetBool


DIMENSION_WIDTH = 100
DIMENSION_HEIGHT = 120


@pytest.mark.rostest
def generate_test_description():
    nitros_image_switch_node = ComposableNode(
        name='nitros_image_switch_node',
        package='custom_nitros_image',
        plugin='nvidia::isaac_ros::custom_nitros_image::NitrosImageSwitchNode',
        namespace=NitrosImageSwitchNodeTest.generate_namespace(),
        parameters=[{'sync_queue_size': 100}]
    )

    return NitrosImageSwitchNodeTest.generate_test_description([
        ComposableNodeContainer(
            name='container',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[nitros_image_switch_node],
            namespace=NitrosImageSwitchNodeTest.generate_namespace(),
            output='screen',
        )
    ])


class NitrosImageSwitchNodeTest(IsaacROSBaseTest):
    filepath = pathlib.Path(os.path.dirname(__file__))

    def test_nitros_image_switch(self):
        """Test that switch ON passes messages through and switch OFF blocks them."""
        received_messages = {}

        self.generate_namespace_lookup(
            ['image', 'camera_info', 'switched_image', 'switched_camera_info']
        )

        image_pub = self.node.create_publisher(
            Image, self.namespaces['image'], self.DEFAULT_QOS)
        camera_info_pub = self.node.create_publisher(
            CameraInfo, self.namespaces['camera_info'], self.DEFAULT_QOS)

        subs = self.create_logging_subscribers(
            [('switched_image', Image), ('switched_camera_info', CameraInfo)],
            received_messages, accept_multiple_messages=True)

        switch_client = self.node.create_client(SetBool, '/isaac_ros_test/switch')
        while not switch_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for switch service...')

        # Create test image
        cv_image = np.zeros((DIMENSION_HEIGHT, DIMENSION_WIDTH, 3), np.uint8)
        cv_image[:] = (1, 2, 3)
        image = CvBridge().cv2_to_imgmsg(cv_image)
        image.encoding = 'bgr8'
        camera_info = CameraInfo()

        def call_switch(state: bool):
            future = switch_client.call_async(SetBool.Request(data=state))
            rclpy.spin_until_future_complete(self.node, future)
            self.assertTrue(future.result().success)

        def publish_message(frame_id: str):
            image.header.stamp = self.node.get_clock().now().to_msg()
            image.header.frame_id = frame_id
            camera_info.header = image.header
            image_pub.publish(image)
            camera_info_pub.publish(camera_info)

        def spin_and_wait(duration: float = 0.5):
            end_time = time.time() + duration
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.05)

        try:
            # Switch ON: publish messages that should pass through
            call_switch(True)
            for i in range(5):
                publish_message(f'on_{i}')
            spin_and_wait()

            on_count = len(received_messages.get('switched_image', []))
            self.assertGreaterEqual(on_count, 5, f'Switch ON: expected 5 messages, got {on_count}')

            # Switch OFF: publish messages that should be blocked
            call_switch(False)
            for i in range(5):
                publish_message(f'off_{i}')
            spin_and_wait()

            off_count = len(received_messages['switched_image'])
            extra = off_count - on_count
            self.assertEqual(off_count, on_count,
                             f'Switch OFF: messages should be blocked, got {extra} extra')

            # Verify all received messages are from the ON state
            for received_image in received_messages['switched_image']:
                self.assertEqual(received_image.height, DIMENSION_HEIGHT)
                self.assertEqual(received_image.width, DIMENSION_WIDTH)
                self.assertEqual(received_image.encoding, 'bgr8')
                self.assertTrue(received_image.header.frame_id.startswith('on_'),
                                f'Unexpected frame_id: {received_image.header.frame_id}')
                self.assertTrue((received_image.data == cv_image.flatten()).all())

            self.assertEqual(len(received_messages['switched_camera_info']), off_count)

        finally:
            self.node.destroy_subscription(subs)
            self.node.destroy_publisher(image_pub)
            self.node.destroy_publisher(camera_info_pub)
