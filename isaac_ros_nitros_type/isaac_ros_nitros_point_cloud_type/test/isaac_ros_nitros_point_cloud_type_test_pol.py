# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

"""Proof-of-Life test for the NitrosPointCloud type adapter."""

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
    """Generate launch description with all ROS 2 nodes for testing."""
    test_ns = IsaacROSNitrosPointCloudTest.generate_namespace()
    container = ComposableNodeContainer(
        name='point_cloud_container',
        namespace='isaac_ros_nitros_container',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_nitros_point_cloud_type',
                plugin='nvidia::isaac_ros::nitros::NitrosPointCloudForwardNode',
                name='NitrosPointCloudForwardNode',
                namespace=test_ns,
                parameters=[{
                    'compatible_format': 'nitros_point_cloud'
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

    return IsaacROSNitrosPointCloudTest.generate_test_description([
        container,
        launch.actions.TimerAction(
            period=2.5, actions=[launch_testing.actions.ReadyToTest()])
    ])


class IsaacROSNitrosPointCloudTest(IsaacROSBaseTest):
    """Validate NitrosPointCloud type adapter."""

    filepath = pathlib.Path(os.path.dirname(__file__))

    @IsaacROSBaseTest.for_each_test_case(subfolder='nitros_point_cloud')
    def test_nitros_point_cloud_type_conversions(self, test_folder) -> None:
        """Expect the image received from NitrosImage type conversion to be identical to source."""
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
                done, "Didn't receive output on output_image topic!")

            received_points = received_messages['output']

            self.assertEqual(cloud_msg.header.frame_id, received_points.header.frame_id,
                             'Source and received frame ids dont match')

            self.assertEqual(len(cloud_msg.data), len(received_points.data),
                             'Source and received image sizes do not match: ' +
                             f'{len(cloud_msg.data)} != {len(received_points.data)}')
            self.assertEqual(cloud_msg.is_bigendian, received_points.is_bigendian,
                             'Source and received image is_bigendian field do not match')
            self.assertEqual(cloud_msg.row_step, received_points.row_step,
                             'Source and received image row_step field do not match')
            self.assertEqual(cloud_msg.point_step, received_points.point_step,
                             'Source and received image point_step field do not match')
            for i in range(len(cloud_msg.data)):
                # by convention the 15th bit is not used and hence not passed through
                if (i % 16 != 15):
                    self.assertEqual(cloud_msg.data[i], received_points.data[i],
                                     'Source and received image pixels do not match')

            print('Source and received images are identical.')
        finally:
            self.node.destroy_subscription(received_cloud_sub)
            self.node.destroy_publisher(cloud_pub)
