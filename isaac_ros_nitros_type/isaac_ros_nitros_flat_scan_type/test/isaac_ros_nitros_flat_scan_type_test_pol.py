# Copyright (c) 2023, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

"""Proof-of-Life test for the NitrosFlatScan type adapter."""

import os
import pathlib
import time

from isaac_ros_pointcloud_interfaces.msg import FlatScan

from isaac_ros_test import IsaacROSBaseTest, JSONConversion

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import pytest
import rclpy


@pytest.mark.rostest
def generate_test_description():
    """Generate launch description with all ROS 2 nodes for testing."""
    test_ns = IsaacROSNitrosFlatScanTest.generate_namespace()
    container = ComposableNodeContainer(
        name='test_container',
        namespace='isaac_ros_nitros_container',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_nitros_flat_scan_type',
                plugin='nvidia::isaac_ros::nitros::NitrosFlatScanForwardNode',
                name='NitrosFlatScanForwardNode',
                namespace=test_ns,
                parameters=[{
                    'compatible_format': 'nitros_flat_scan'
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

    return IsaacROSNitrosFlatScanTest.generate_test_description(
        [container],
        node_startup_delay=2.5
    )


class IsaacROSNitrosFlatScanTest(IsaacROSBaseTest):
    """Validate NitrosFlatScan type adapter."""

    filepath = pathlib.Path(os.path.dirname(__file__))

    @IsaacROSBaseTest.for_each_test_case()
    def test_nitros_flat_scan_type_conversions(self, test_folder) -> None:
        """Expect the flatscan from NitrosFlatScan type to be compatible with source."""
        self.generate_namespace_lookup(['input', 'output'])
        received_messages = {}

        received_message_sub = self.create_logging_subscribers(
            subscription_requests=[('output', FlatScan)],
            received_messages=received_messages
        )

        flat_scan_pub = self.node.create_publisher(
            FlatScan, self.namespaces['input'], self.DEFAULT_QOS)

        try:
            flat_scan = self.load_flat_scan_from_json(
                test_folder / 'flat_scan.json')

            # Wait at most TIMEOUT seconds for subscriber to respond
            TIMEOUT = 2
            end_time = time.time() + TIMEOUT

            done = False
            while time.time() < end_time:
                timestamp = self.node.get_clock().now().to_msg()
                flat_scan.header.stamp = timestamp

                flat_scan_pub.publish(flat_scan)

                rclpy.spin_once(self.node, timeout_sec=0.1)

                # If we have received a message on the output topic, break
                if 'output' in received_messages:
                    done = True
                    break

            self.assertTrue(done, "Didn't receive output on the output topic!")

            received_flat_scan = received_messages['output']

            # Header
            # Note: Since intensities and min range is not present in the
            # gxf message, passthrough of these values are not tested
            # Frame ID is to be passed from NitrosFlatScan to ROS message
            self.assertEqual(flat_scan.header.frame_id,
                             received_flat_scan.header.frame_id)
            # Max range to be passed through
            self.assertEqual(flat_scan.range_max,
                             received_flat_scan.range_max)

            # Array size check
            self.assertEqual(len(flat_scan.ranges), len(
                received_flat_scan.ranges), 'Number of ranges does not match')
            self.assertEqual(len(flat_scan.angles), len(
                received_flat_scan.angles), 'Number of angles does not match')

            # Array values check
            for beam_range, received_range in zip(flat_scan.ranges, received_flat_scan.ranges):
                self.assertEqual(
                    beam_range, received_range,
                    'Range array does not match')
            for angle, received_angle in zip(flat_scan.angles, received_flat_scan.angles):
                self.assertEqual(
                    angle, received_angle,
                    'Angle array does not match')

            print('The received flatscan is verified successfully')
        finally:
            self.node.destroy_subscription(received_message_sub)
            self.node.destroy_publisher(flat_scan_pub)

    @staticmethod
    def load_flat_scan_from_json(
            json_filepath: pathlib.Path) -> FlatScan:
        """
        Load a FlatScan message from a JSON filepath.

        Parameters
        ----------
        json_filepath : Path
            The path to a JSON file containing the FlatScan fields

        Returns
        -------
        FlatScan
            Generated FlatScan message

        """
        flatscan_json = JSONConversion.load_from_json(
            json_filepath)

        flat_scan_msg = FlatScan()
        flat_scan_msg.header.frame_id = flatscan_json[
            'header']['frame_id']
        flat_scan_msg.header.stamp.sec = flatscan_json[
            'header']['stamp']['sec']
        flat_scan_msg.header.stamp.nanosec = flatscan_json[
            'header']['stamp']['nanosec']
        flat_scan_msg.range_max = flatscan_json[
            'range_max']
        flat_scan_msg.range_min = flatscan_json[
            'range_min']
        for beam_range in flatscan_json['ranges']:
            flat_scan_msg.ranges.append(beam_range)
        for angle in flatscan_json['angles']:
            flat_scan_msg.angles.append(angle)
        return flat_scan_msg
