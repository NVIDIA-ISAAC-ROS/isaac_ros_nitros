# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

"""Proof-of-Life test for the NitrosOccupancyGrid type adapter."""

import os
import pathlib
import time

from isaac_ros_test import IsaacROSBaseTest, JSONConversion

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from nav_msgs.msg import OccupancyGrid

import pytest
import rclpy


@pytest.mark.rostest
def generate_test_description():
    """Generate launch description with all ROS 2 nodes for testing."""
    test_ns = IsaacROSNitrosOccupancyGridTest.generate_namespace()
    container = ComposableNodeContainer(
        name='test_container',
        namespace='isaac_ros_nitros_container',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_nitros_occupancy_grid_type',
                plugin='nvidia::isaac_ros::nitros::NitrosOccupancyGridForwardNode',
                name='NitrosOccupancyGridForwardNode',
                namespace=test_ns,
                parameters=[{
                    'compatible_format': 'nitros_occupancy_grid'
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

    return IsaacROSNitrosOccupancyGridTest.generate_test_description(
        [container],
        node_startup_delay=2.5
    )


class IsaacROSNitrosOccupancyGridTest(IsaacROSBaseTest):
    """Validate NitrosOccupancyGrid type adapter."""

    filepath = pathlib.Path(os.path.dirname(__file__))

    @IsaacROSBaseTest.for_each_test_case()
    def test_nitros_occupancy_grid_type_conversions(self, test_folder) -> None:
        """Expect the occupancy grid from NitrosOccupancyGrid type to be compatible with source."""
        self.generate_namespace_lookup(['input', 'output'])
        received_messages = {}

        received_message_sub = self.create_logging_subscribers(
            subscription_requests=[('output', OccupancyGrid)],
            received_messages=received_messages
        )

        occupancy_grid_pub = self.node.create_publisher(
            OccupancyGrid, self.namespaces['input'], self.DEFAULT_QOS)

        try:
            occupancy_grid: OccupancyGrid = JSONConversion.load_occupancy_grid_from_json(
                test_folder / 'occupancy_grid.json')

            # Wait at most TIMEOUT seconds for subscriber to respond
            TIMEOUT = 2
            end_time = time.time() + TIMEOUT

            done = False
            while time.time() < end_time:
                timestamp = self.node.get_clock().now().to_msg()
                occupancy_grid.header.stamp = timestamp

                occupancy_grid_pub.publish(occupancy_grid)

                rclpy.spin_once(self.node, timeout_sec=0.1)

                # If we have received a message on the output topic, break
                if 'output' in received_messages:
                    done = True
                    break

            self.assertTrue(done, "Didn't receive output on the output topic!")

            received_occupancy_grid = received_messages['output']

            # Header
            # Frame ID is to be passed from NitrosOccupancyGrid to ROS message
            # self.assertEqual(
            #   occupancy_grid.header.frame_id, received_occupancy_grid.header.frame_id)

            # MapMetadata
            self.assertEqual(occupancy_grid.info.resolution,
                             received_occupancy_grid.info.resolution, 'Resolution does not match')
            self.assertEqual(occupancy_grid.info.width,
                             received_occupancy_grid.info.width, 'Width does not match')
            self.assertEqual(occupancy_grid.info.height,
                             received_occupancy_grid.info.height, 'Height does not match')
            self.assertEqual(
                round(occupancy_grid.info.origin.position.x, 2),
                round(received_occupancy_grid.info.origin.position.x, 2),
                'Position x value does not match')
            self.assertEqual(
                round(occupancy_grid.info.origin.position.y, 2),
                round(received_occupancy_grid.info.origin.position.y, 2),
                'Position y value does not match')
            self.assertEqual(
                round(occupancy_grid.info.origin.position.z, 2),
                round(received_occupancy_grid.info.origin.position.z, 2),
                'Position z value does not match')

            self.assertEqual(
                round(occupancy_grid.info.origin.orientation.x, 2),
                round(received_occupancy_grid.info.origin.orientation.x, 2),
                'Orientation x value does not match')
            self.assertEqual(
                round(occupancy_grid.info.origin.orientation.y, 2),
                round(received_occupancy_grid.info.origin.orientation.y, 2),
                'Orientation x value does not match')
            self.assertEqual(
                round(occupancy_grid.info.origin.orientation.z, 2),
                round(received_occupancy_grid.info.origin.orientation.z, 2),
                'Orientation x value does not match')
            self.assertEqual(
                round(occupancy_grid.info.origin.orientation.w, 2),
                round(received_occupancy_grid.info.origin.orientation.w, 2),
                'Orientation w value does not match')

            # data
            for cell, received_cell in zip(occupancy_grid.data, received_occupancy_grid.data):
                self.assertEqual(cell, received_cell, 'Occupancy data does not match')

            print('The received occupancy grid is verified successfully')
        finally:
            self.node.destroy_subscription(received_message_sub)
            self.node.destroy_publisher(occupancy_grid_pub)
