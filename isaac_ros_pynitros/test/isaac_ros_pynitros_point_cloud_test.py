# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
"""Proof-of-Life test for the PyNITROS point cloud type."""

import os
import pathlib
import subprocess
import time

from isaac_ros_test import IsaacROSBaseTest, PCDLoader
import launch

from launch_ros.actions import Node
import launch_testing
import pytest
import rclpy

from sensor_msgs.msg import PointCloud2


class IsaacROSPyNitrosPointCloudTest(IsaacROSBaseTest):
    """Validate Nitros Bridge on Point Cloud type."""

    filepath = pathlib.Path(os.path.dirname(__file__))
    skip_test = False  # Initialize the class attribute

    @IsaacROSBaseTest.for_each_test_case(subfolder='nitros_point_cloud')
    def test_pynitros_point_cloud(self, test_folder) -> None:
        if IsaacROSPyNitrosPointCloudTest.skip_test:
            self.skipTest('No ptrace permission! Skipping test.')
        else:
            IsaacROSPyNitrosPointCloudTest.DEFAULT_NAMESPACE = 'pynitros'
            self.generate_namespace_lookup(['pynitros1_input_msg', 'pynitros2_output_msg_ros'])
            received_messages = {}

            received_point_cloud_sub = self.create_logging_subscribers(
                subscription_requests=[
                    ('pynitros2_output_msg_ros', PointCloud2)],
                received_messages=received_messages)

            point_cloud_pub = self.node.create_publisher(
                PointCloud2, self.namespaces['pynitros1_input_msg'],
                self.DEFAULT_QOS)

            try:
                # Use the standard PCDLoader approach like other PointCloud2
                # tests
                point_cloud = PCDLoader.generate_pointcloud2_from_pcd_file(
                    test_folder / 'ketchup.pcd', 'test_frame')
                point_cloud.header.stamp = self.node.get_clock().now().to_msg()

                # Wait at most TIMEOUT seconds for subscriber to respond
                TIMEOUT = 30
                end_time = time.time() + TIMEOUT

                done = False
                while time.time() < end_time:
                    point_cloud_pub.publish(point_cloud)
                    rclpy.spin_once(self.node, timeout_sec=0.1)

                    # If we have received a message on the output topic, break
                    if 'pynitros2_output_msg_ros' in received_messages:
                        done = True
                        break

                self.assertTrue(
                    done,
                    "Didn't receive output on output_point_cloud topic!")

                received_point_cloud = received_messages['pynitros2_output_msg_ros']

                self.assertEqual(
                    str(point_cloud.header.stamp),
                    str(received_point_cloud.header.stamp),
                    'Timestamps do not match.')

                self.assertEqual(
                    len(point_cloud.data), len(received_point_cloud.data),
                    'Source and received point cloud sizes do not match: ' +
                    f'{len(point_cloud.data)} != '
                    f'{len(received_point_cloud.data)}')

                # Make sure that the source and received point clouds are the same
                self.assertEqual(received_point_cloud.height, point_cloud.height,
                                 'Source and received point cloud heights do not match: ' +
                                 f'{point_cloud.height} != {received_point_cloud.height}')
                self.assertEqual(received_point_cloud.width, point_cloud.width,
                                 'Source and received point cloud widths do not match: ' +
                                 f'{point_cloud.width} != {received_point_cloud.width}')
                self.assertEqual(received_point_cloud.is_bigendian, point_cloud.is_bigendian,
                                 'Source and received point cloud is_bigendian do not match: ' +
                                 f'{point_cloud.is_bigendian} != '
                                 f'{received_point_cloud.is_bigendian}')
                self.assertEqual(received_point_cloud.point_step, point_cloud.point_step,
                                 'Source and received point cloud point_step do not match: ' +
                                 f'{point_cloud.point_step} != {received_point_cloud.point_step}')
                self.assertEqual(received_point_cloud.row_step, point_cloud.row_step,
                                 'Source and received point cloud row_step do not match: ' +
                                 f'{point_cloud.row_step} != {received_point_cloud.row_step}')
                self.assertEqual(received_point_cloud.is_dense, point_cloud.is_dense,
                                 'Source and received point cloud is_dense do not match: ' +
                                 f'{point_cloud.is_dense} != {received_point_cloud.is_dense}')
                self.assertEqual(received_point_cloud.fields, point_cloud.fields,
                                 'Source and received point cloud fields do not match: ' +
                                 f'{point_cloud.fields} != {received_point_cloud.fields}')
                for i in range(len(point_cloud.data)):
                    self.assertEqual(
                        point_cloud.data[i], received_point_cloud.data[i],
                        'Source and received point cloud data do not match')

                print('Source and receive point cloud match!')

            finally:
                self.node.destroy_subscription(received_point_cloud_sub)
                self.node.destroy_publisher(point_cloud_pub)


@pytest.mark.rostest
def generate_test_description():
    """Generate launch description with all ROS 2 nodes for testing."""
    ptrace_result = subprocess.run(
        ['cat', '/proc/sys/kernel/yama/ptrace_scope'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True)
    pidfd_getfd_result = subprocess.run(
        ['grep', 'pidfd_getfd', '/proc/kallsyms'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True)
    if (ptrace_result.stdout.strip() == '0' and pidfd_getfd_result.returncode == 0):
        IsaacROSPyNitrosPointCloudTest.skip_test = False
        pynitros_point_cloud_forward_node_1 = Node(
            name='pynitros_point_cloud_forward_node_1',
            package='isaac_ros_pynitros',
            namespace='pynitros',
            executable='pynitros_point_cloud_forward_node',
            parameters=[{
                'enable_ros_subscribe': True,
                'enable_ros_publish': False
            }],
            remappings=[
                ('topic_forward_input_ros', 'pynitros1_input_msg'),
                ('topic_forward_output', 'pynitros1_output_msg'),
                ('topic_forward_output_ros', 'pynitros1_output_msg_ros')
            ],
            output='screen')

        pynitros_point_cloud_forward_node_2 = Node(
            name='pynitros_point_cloud_forward_node_2',
            package='isaac_ros_pynitros',
            executable='pynitros_point_cloud_forward_node',
            namespace='pynitros',
            parameters=[{
                'enable_ros_subscribe': False,
                'enable_ros_publish': True
            }],
            remappings=[
                ('topic_forward_input', 'pynitros1_output_msg'),
                ('topic_forward_output', 'pynitros2_output_msg'),
                ('topic_forward_output_ros', 'pynitros2_output_msg_ros')
            ],
            output='screen')

        return IsaacROSPyNitrosPointCloudTest.generate_test_description([
            pynitros_point_cloud_forward_node_1,
            pynitros_point_cloud_forward_node_2,
            launch.actions.TimerAction(
                period=2.5,
                actions=[launch_testing.actions.ReadyToTest()])
        ])
    else:
        IsaacROSPyNitrosPointCloudTest.skip_test = True
        return IsaacROSPyNitrosPointCloudTest.generate_test_description([
            launch_testing.actions.ReadyToTest()])
