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
"""Proof-of-Life test for the PyNITROS message filter."""

import os
import pathlib
import subprocess
import time

from isaac_ros_test import IsaacROSBaseTest, JSONConversion
import launch

from launch_ros.actions import Node
import launch_testing
import pytest
import rclpy

from sensor_msgs.msg import CameraInfo, Image


@pytest.mark.rostest
def generate_test_description():
    """Generate launch description with all ROS 2 nodes for testing."""
    ptace_result = subprocess.run(['cat', '/proc/sys/kernel/yama/ptrace_scope'],
                                  stdout=subprocess.PIPE,
                                  stderr=subprocess.PIPE,
                                  text=True)
    pidfd_getfd_result = subprocess.run(['grep', 'pidfd_getfd', '/proc/kallsyms'],
                                        stdout=subprocess.PIPE,
                                        stderr=subprocess.PIPE,
                                        text=True)
    if (ptace_result.stdout.strip() == '0' and pidfd_getfd_result.returncode == 0):
        IsaacROSPyNitrosTest.skip_test = False
        pynitros_image_forward_node = Node(name='pynitros_image_forward_node',
                                           package='isaac_ros_pynitros',
                                           namespace='pynitros',
                                           executable='pynitros_image_forward_node',
                                           parameters=[{
                                                'enable_ros_subscribe': True,
                                                'enable_ros_publish': False
                                           }],
                                           remappings=[
                                                ('pynitros_input_msg', 'pynitros1_input_msg'),
                                                ('pynitros_output_msg', 'pynitros1_output_msg'),
                                           ],
                                           output='screen')
        pynitros_sync_node = Node(name='pynitros_sync_node',
                                  package='isaac_ros_pynitros',
                                  namespace='pynitros',
                                  executable='pynitros_message_filter_sync_node',
                                  parameters=[{
                                        'enable_ros_subscribe': False,
                                        'enable_ros_publish': True
                                   }],
                                  remappings=[
                                        ('image', 'pynitros1_output_msg')
                                  ],
                                  output='screen')

        return IsaacROSPyNitrosTest.generate_test_description([
            pynitros_image_forward_node,
            pynitros_sync_node,
            launch.actions.TimerAction(period=2.5, actions=[launch_testing.actions.ReadyToTest()])
        ])
    else:
        IsaacROSPyNitrosTest.skip_test = True
        return IsaacROSPyNitrosTest.generate_test_description(
            [launch_testing.actions.ReadyToTest()])


class IsaacROSPyNitrosTest(IsaacROSBaseTest):
    """Validate Nitros Bridge on Image type."""

    filepath = pathlib.Path(os.path.dirname(__file__))

    @IsaacROSBaseTest.for_each_test_case(subfolder='nitros_image')
    def test_pynitros_image(self, test_folder) -> None:
        if self.skip_test:
            self.skipTest('No ptrace permission! Skipping test.')
        else:
            IsaacROSPyNitrosTest.DEFAULT_NAMESPACE = 'pynitros'
            self.generate_namespace_lookup(['pynitros1_input_msg',
                                            'camera_info',
                                            'synced_output_ros'])
            received_messages = {}

            received_image_sub = self.create_logging_subscribers(
                subscription_requests=[('synced_output_ros', Image)],
                received_messages=received_messages)

            image_pub = self.node.create_publisher(Image,
                                                   self.namespaces['pynitros1_input_msg'],
                                                   self.DEFAULT_QOS)
            camera_info_pub = self.node.create_publisher(CameraInfo,
                                                         self.namespaces['camera_info'],
                                                         self.DEFAULT_QOS)
            try:
                image = JSONConversion.load_image_from_json(test_folder / 'image.json')
                timestamp = self.node.get_clock().now().to_msg()
                image.header.stamp = timestamp

                camera_info = CameraInfo()
                camera_info.header.stamp = timestamp
                # Wait at most TIMEOUT seconds for subscriber to respond
                TIMEOUT = 30
                end_time = time.time() + TIMEOUT

                done = False
                while time.time() < end_time:
                    image_pub.publish(image)
                    camera_info_pub.publish(camera_info)
                    rclpy.spin_once(self.node, timeout_sec=0.1)

                    # If we have received a message on the output topic, break
                    if 'synced_output_ros' in received_messages:
                        done = True
                        break

                self.assertTrue(done, "Didn't receive output on output_image topic!")

                received_image = received_messages['synced_output_ros']

                self.assertEqual(str(timestamp), str(received_image.header.stamp),
                                 'Timestamps do not match.')

                self.assertEqual(
                    len(image.data), len(received_image.data),
                    'Source and received image sizes do not match: ' +
                    f'{len(image.data)} != {len(received_image.data)}')

                # Make sure that the source and received images are the same
                self.assertEqual(
                    received_image.height, image.height,
                    'Source and received image heights do not match: ' +
                    f'{image.height} != {received_image.height}')
                self.assertEqual(
                    received_image.width, image.width,
                    'Source and received image widths do not match: ' +
                    f'{image.width} != {received_image.width}')

                for i in range(len(image.data)):
                    self.assertEqual(image.data[i], received_image.data[i],
                                     'Source and received image pixels do not match')

            finally:
                self.node.destroy_subscription(received_image_sub)
                self.node.destroy_publisher(image_pub)