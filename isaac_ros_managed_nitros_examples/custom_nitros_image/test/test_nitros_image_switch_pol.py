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
    nitros_image_Switch_node = ComposableNode(
        name='nitros_image_switch_node',
        package='custom_nitros_image',
        plugin='nvidia::isaac_ros::custom_nitros_image::NitrosImageSwitchNode',
        namespace=NitrosImageSwitchNodeTest.generate_namespace(),
        parameters=[{
            'sync_queue_size': 100,  # Ensure no messages dropped
        }]
    )

    return NitrosImageSwitchNodeTest.generate_test_description([
        ComposableNodeContainer(
            name='container',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[nitros_image_Switch_node],
            namespace=NitrosImageSwitchNodeTest.generate_namespace(),
            output='screen',
        )
    ])


class NitrosImageSwitchNodeTest(IsaacROSBaseTest):
    filepath = pathlib.Path(os.path.dirname(__file__))

    def test_nitros_image_switch(self):
        TIMEOUT = 300
        NUM_EXPECTED_MSGS = 10
        received_messages = {}

        self.generate_namespace_lookup(
            ['image', 'camera_info', 'switched_image', 'switched_camera_info']
        )

        image_pub = self.node.create_publisher(
            Image, self.namespaces['image'], self.DEFAULT_QOS)

        camera_info_pub = self.node.create_publisher(
            CameraInfo, self.namespaces['camera_info'], self.DEFAULT_QOS
        )

        subs = self.create_logging_subscribers(
            [('switched_image', Image),
             ('switched_camera_info', CameraInfo),
             ], received_messages, accept_multiple_messages=True)

        self.cli = self.node.create_client(
            SetBool, '/isaac_ros_test/switch'
        )
        self.req = SetBool.Request()

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')

        try:
            # Create white image
            cv_image = np.zeros((DIMENSION_HEIGHT, DIMENSION_WIDTH, 3), np.uint8)
            cv_image[:] = (1, 2, 3)
            image = CvBridge().cv2_to_imgmsg(cv_image)
            image.encoding = 'bgr8'
            camera_info = CameraInfo()

            end_time = time.time() + TIMEOUT
            done = False

            # Switch between triggering and not triggering
            # Each even number will be received, odd numbers won't be
            count = 0
            self.req.data = False
            while time.time() < end_time:
                self.req.data = not self.req.data
                self.future = self.cli.call_async(self.req)
                rclpy.spin_until_future_complete(self.node, self.future)

                image.header.stamp = self.node.get_clock().now().to_msg()
                image.header.frame_id = str(count)
                camera_info.header = image.header
                image_pub.publish(image)
                camera_info_pub.publish(camera_info)
                count += 1
                rclpy.spin_once(self.node, timeout_sec=(0.1))
                if 'switched_image' in received_messages and \
                        len(received_messages['switched_image']) >= NUM_EXPECTED_MSGS and \
                        'switched_camera_info' in received_messages and \
                        len(received_messages['switched_camera_info']) >= NUM_EXPECTED_MSGS:
                    done = True
                    break
            self.assertTrue(done, 'Appropriate output not received')
            images = received_messages['switched_image']
            self.assertEqual(len(images), NUM_EXPECTED_MSGS)
            self.assertEqual(len(received_messages['switched_camera_info']), NUM_EXPECTED_MSGS)

            for i, received_image in enumerate(images):
                self.assertEqual(received_image.height, DIMENSION_HEIGHT)
                self.assertEqual(received_image.width, DIMENSION_WIDTH)
                self.assertEqual(received_image.encoding, 'bgr8')
                self.assertEqual(int(received_image.header.frame_id) % 2, 0)
                self.assertTrue((received_image.data == cv_image.flatten()).all())

        finally:
            self.node.destroy_subscription(subs)
            self.node.destroy_publisher(image_pub)
