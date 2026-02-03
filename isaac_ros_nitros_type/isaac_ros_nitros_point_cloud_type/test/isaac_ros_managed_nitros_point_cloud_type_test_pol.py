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

"""Proof-of-Life test for ManagedNITROS with NitrosPointCloud type adapter."""

import os
import pathlib
import time

from isaac_ros_test import IsaacROSBaseTest
from isaac_ros_test.pcd_loader import PCDLoader

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import launch_testing

import pytest
import rclpy
from sensor_msgs.msg import PointCloud2


@pytest.mark.rostest
def generate_test_description():
    """Generate launch description with ManagedNITROS forward node for testing."""
    test_ns = IsaacROSManagedNitrosPointCloudTest.generate_namespace()
    container = ComposableNodeContainer(
        name='managed_nitros_point_cloud_container',
        namespace='isaac_ros_nitros_container',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_nitros_point_cloud_type',
                plugin='nvidia::isaac_ros::nitros::ManagedNitrosPointCloudForwardNode',
                name='ManagedNitrosPointCloudForwardNode',
                namespace=test_ns,
                remappings=[
                    (test_ns+'/topic_forward_input', test_ns+'/input'),
                    (test_ns+'/topic_forward_output', test_ns+'/output'),
                ]
            ),
        ],
        output='both',
        arguments=['--ros-args', '--log-level', 'info'],
    )

    return IsaacROSManagedNitrosPointCloudTest.generate_test_description([
        container,
        launch.actions.TimerAction(
            period=2.5, actions=[launch_testing.actions.ReadyToTest()])
    ])


class IsaacROSManagedNitrosPointCloudTest(IsaacROSBaseTest):
    """Validate ManagedNITROS with NitrosPointCloud type adapter."""

    filepath = pathlib.Path(os.path.dirname(__file__))

    @IsaacROSBaseTest.for_each_test_case(subfolder='nitros_point_cloud')
    def test_managed_nitros_point_cloud_type_conversions(self, test_folder) -> None:
        """Expect the point cloud received via ManagedNITROS to be identical to source."""
        self.generate_namespace_lookup(['input', 'output'])
        received_messages = {}

        received_cloud_sub = self.create_logging_subscribers(
            subscription_requests=[('output', PointCloud2)],
            received_messages=received_messages
        )

        cloud_pub = self.node.create_publisher(
            PointCloud2, self.namespaces['input'], self.DEFAULT_QOS)

        try:
            cloud_msg = PCDLoader.generate_pointcloud2_from_pcd_file(
                test_folder / 'ketchup.pcd', 'sample_points')
            # Wait at most TIMEOUT seconds for subscriber to respond
            TIMEOUT = 2
            end_time = time.time() + TIMEOUT

            done = False
            while time.time() < end_time:
                timestamp = self.node.get_clock().now().to_msg()
                cloud_msg.header.stamp = timestamp

                cloud_pub.publish(cloud_msg)
                rclpy.spin_once(self.node, timeout_sec=0.1)

                # If we have received a message on the output topic, break
                if 'output' in received_messages:
                    done = True
                    break

            self.assertTrue(
                done, "Didn't receive output on output cloud topic!")

            received_points = received_messages['output']

            # Validate header fields
            self.assertEqual(cloud_msg.header.frame_id, received_points.header.frame_id,
                             'Source and received frame ids dont match')

            self.assertEqual(cloud_msg.header.stamp.sec, received_points.header.stamp.sec,
                             f'Source and received timestamp seconds do not match: '
                             f'{cloud_msg.header.stamp.sec} != {received_points.header.stamp.sec}')

            self.assertEqual(cloud_msg.header.stamp.nanosec, received_points.header.stamp.nanosec,
                             f'Source and received timestamp nanoseconds do not match: '
                             f'{cloud_msg.header.stamp.nanosec} != '
                             f'{received_points.header.stamp.nanosec}')

            # Validate point cloud structure
            self.assertEqual(len(cloud_msg.data), len(received_points.data),
                             'Source and received point cloud sizes do not match: ' +
                             f'{len(cloud_msg.data)} != {len(received_points.data)}')
            self.assertEqual(cloud_msg.is_bigendian, received_points.is_bigendian,
                             'Source and received point cloud is_bigendian field do not match')
            self.assertEqual(cloud_msg.row_step, received_points.row_step,
                             'Source and received point cloud row_step field do not match')
            self.assertEqual(cloud_msg.point_step, received_points.point_step,
                             'Source and received point cloud point_step field do not match')
            self.assertEqual(cloud_msg.height, received_points.height,
                             'Source and received point cloud height field do not match: '
                             f'{cloud_msg.height} != {received_points.height}')
            self.assertEqual(cloud_msg.width, received_points.width,
                             'Source and received point cloud width field do not match: '
                             f'{cloud_msg.width} != {received_points.width}')
            self.assertEqual(cloud_msg.is_dense, received_points.is_dense,
                             'Source and received point cloud is_dense field do not match')

            # Validate PointFields
            self.assertEqual(len(cloud_msg.fields), len(received_points.fields),
                             'Source and received point cloud field counts do not match: '
                             f'{len(cloud_msg.fields)} != {len(received_points.fields)}')
            for i, (src_field, recv_field) in enumerate(
                    zip(cloud_msg.fields, received_points.fields)):
                self.assertEqual(src_field.name, recv_field.name,
                                 f'Field {i} name mismatch: '
                                 f'{src_field.name} != {recv_field.name}')
                self.assertEqual(src_field.offset, recv_field.offset,
                                 f'Field {i} ({src_field.name}) offset mismatch: '
                                 f'{src_field.offset} != {recv_field.offset}')
                self.assertEqual(src_field.datatype, recv_field.datatype,
                                 f'Field {i} ({src_field.name}) datatype mismatch: '
                                 f'{src_field.datatype} != {recv_field.datatype}')
                self.assertEqual(src_field.count, recv_field.count,
                                 f'Field {i} ({src_field.name}) count mismatch: '
                                 f'{src_field.count} != {recv_field.count}')

            # Validate point data
            for i in range(len(cloud_msg.data)):
                self.assertEqual(cloud_msg.data[i], received_points.data[i],
                                 'Source and received point clouds do not match')

            print('Source and received point clouds are identical.')
        finally:
            self.node.destroy_subscription(received_cloud_sub)
            self.node.destroy_publisher(cloud_pub)
