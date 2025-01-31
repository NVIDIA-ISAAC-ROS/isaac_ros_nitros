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
import time

from isaac_ros_test import IsaacROSBaseTest

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import pytest
import rclpy
from sensor_msgs.msg import CameraInfo, Image


IMAGE_HEIGHT = 100
IMAGE_WIDTH = 150
TOTAL_COUNT = 10
DROP_COUNT = 7
RECEIVE_WAIT_TIME = 2


@pytest.mark.rostest
def generate_test_description():
    """Generate launch description with all ROS 2 nodes for testing."""
    nitros_drop_node = ComposableNode(
        package='isaac_ros_nitros_topic_tools',
        plugin='nvidia::isaac_ros::nitros::NitrosCameraDropNode',
        name='nitros_drop_node',
        namespace=NitrosCameraDropNodeMode1Test.generate_namespace(),
        parameters=[{
                    'X': DROP_COUNT,
                    'Y': TOTAL_COUNT,
                    'mode': 'mono+depth'
                    }]
    )

    container = ComposableNodeContainer(
        name='test_container',
        namespace='isaac_ros_nitros_container',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[nitros_drop_node],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
    )

    return NitrosCameraDropNodeMode1Test.generate_test_description([container])


class NitrosCameraDropNodeMode1Test(IsaacROSBaseTest):
    """Test NitrosCameraDropNode in Mode = mono+depth."""

    def create_image(self, name):
        image = Image()
        image.height = IMAGE_HEIGHT
        image.width = IMAGE_WIDTH
        image.encoding = 'rgb8'
        image.is_bigendian = False
        image.step = IMAGE_WIDTH * 3
        image.data = [0] * IMAGE_HEIGHT * IMAGE_WIDTH * 3
        image.header.frame_id = name
        return image

    def test_drop_node(self) -> None:
        """
        Test case for the drop_node function.

        This test case verifies the behavior of the drop_node function by publishing a fixed
        number of messages to the input topics and
        checking if the correct number of messages are dropped.
        """
        # Generate namespace lookup
        self.generate_namespace_lookup(
            ['image_1', 'camera_info_1', 'depth_1', 'image_1_drop',
             'camera_info_1_drop', 'depth_1_drop'])

        received_messages = []

        self.create_exact_time_sync_logging_subscribers(
            [('image_1_drop', Image), ('camera_info_1_drop', CameraInfo),
                ('depth_1_drop', Image)],
            received_messages, accept_multiple_messages=True)
        # Publish to inputs
        image_pub = self.node.create_publisher(
            Image, self.namespaces['image_1'], self.DEFAULT_QOS)
        camera_info_pub = self.node.create_publisher(
            CameraInfo, self.namespaces['camera_info_1'], self.DEFAULT_QOS)
        depth_pub = self.node.create_publisher(
            Image, self.namespaces['depth_1'], self.DEFAULT_QOS)
        try:
            image_msg = self.create_image('test_image')
            camera_info_msg = CameraInfo()
            camera_info_msg.distortion_model = 'plumb_bob'
            depth_msg = self.create_image('test_depth')
            self.node.get_logger().info('Starting to publish messages')

            # Publish TOTAL_COUNT number of messages
            counter = 0
            # This test requires deterministic communication between the test node
            # and the NitrosCameraDropNode. A 1-second delay ensures the NitrosCameraDropNode
            # has fully started, preventing message drops/flaky tests.
            time.sleep(1.0)
            while counter < TOTAL_COUNT:
                header = self.node.get_clock().now().to_msg()
                image_msg.header.stamp = header
                camera_info_msg.header.stamp = header
                depth_msg.header.stamp = header
                image_pub.publish(image_msg)
                camera_info_pub.publish(camera_info_msg)
                depth_pub.publish(depth_msg)
                time.sleep(0.1)
                counter += 1
                rclpy.spin_once(self.node, timeout_sec=0.01)

            # Wait for the messages to be received
            end_time = time.time() + RECEIVE_WAIT_TIME
            while time.time() < end_time:
                time.sleep(0.1)
                rclpy.spin_once(self.node, timeout_sec=0.1)

            # Check if the correct number of messages are received
            self.assertTrue(len(received_messages) > 0, 'Did not receive output messages')
            self.assertTrue(len(received_messages) == (TOTAL_COUNT-DROP_COUNT),
                            'Did not receive the correct number of output messages')
        finally:
            self.node.destroy_publisher(image_pub)
            self.node.destroy_publisher(camera_info_pub)
            self.node.destroy_publisher(depth_pub)
