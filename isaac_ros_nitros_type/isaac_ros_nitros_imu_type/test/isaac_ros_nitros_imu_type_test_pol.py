# Copyright (c) 2023, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

"""Proof-of-Life test for the NitrosImu type adapter."""

import os
import pathlib
import time

from isaac_ros_test import IsaacROSBaseTest, JSONConversion

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import launch_testing

import pytest
import rclpy

from sensor_msgs.msg import Imu


@pytest.mark.rostest
def generate_test_description():
    """Generate launch description with all ROS 2 nodes for testing."""
    test_ns = IsaacROSNitrosImuTest.generate_namespace()
    container = ComposableNodeContainer(
        name='test_container',
        namespace='isaac_ros_nitros_container',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_nitros_imu_type',
                plugin='nvidia::isaac_ros::nitros::NitrosImuForwardNode',
                name='NitrosImuForwardNode',
                namespace=test_ns,
                parameters=[{
                    'compatible_format': 'nitros_imu'
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

    return IsaacROSNitrosImuTest.generate_test_description([
        container,
        launch.actions.TimerAction(
            period=2.5, actions=[launch_testing.actions.ReadyToTest()])
    ])


class IsaacROSNitrosImuTest(IsaacROSBaseTest):
    """Validate NitrosImu type adapter."""

    filepath = pathlib.Path(os.path.dirname(__file__))

    @IsaacROSBaseTest.for_each_test_case(subfolder='nitros_imu')
    def test_nitros_imu_type_conversions(self, test_folder) -> None:
        """Expect the tag from NitrosImu type to be compatible with source."""
        self.generate_namespace_lookup(['input', 'output'])
        received_messages = {}

        received_message_sub = self.create_logging_subscribers(
            subscription_requests=[('output', Imu)],
            received_messages=received_messages
        )

        imu_pub = self.node.create_publisher(
            Imu, self.namespaces['input'], self.DEFAULT_QOS)

        try:
            imu = self.load_imu_from_json(test_folder / 'imu.json')

            # Wait at most TIMEOUT seconds for subscriber to respond
            TIMEOUT = 10
            end_time = time.time() + TIMEOUT

            done = False
            while time.time() < end_time:
                timestamp = self.node.get_clock().now().to_msg()
                imu.header.stamp = timestamp

                imu_pub.publish(imu)

                rclpy.spin_once(self.node, timeout_sec=0.1)

                # If we have received a message on the output topic, break
                if 'output' in received_messages:
                    done = True
                    break

            self.assertTrue(done, "Didn't receive output on the output topic!")

            received_imu = received_messages['output']

            # Only test for data passed through gxf
            self.assertEqual(imu.header.stamp.sec,
                             received_imu.header.stamp.sec)
            self.assertEqual(imu.header.stamp.nanosec,
                             received_imu.header.stamp.nanosec)
            self.assertEqual(imu.header.frame_id,
                             received_imu.header.frame_id)
            self.assertEqual(imu.angular_velocity.x,
                             received_imu.angular_velocity.x)
            self.assertEqual(imu.angular_velocity.y,
                             received_imu.angular_velocity.y)
            self.assertEqual(imu.angular_velocity.z,
                             received_imu.angular_velocity.z)
            self.assertEqual(imu.linear_acceleration.x,
                             received_imu.linear_acceleration.x)
            self.assertEqual(imu.linear_acceleration.y,
                             received_imu.linear_acceleration.y)
            self.assertEqual(imu.linear_acceleration.z,
                             received_imu.linear_acceleration.z)
            print('The received imu message has been verified successfully')
        finally:
            self.node.destroy_subscription(received_message_sub)
            self.node.destroy_publisher(imu_pub)

    @staticmethod
    def load_imu_from_json(
            json_filepath: pathlib.Path) -> Imu:
        """
        Load a Imu message from a JSON filepath.

        Parameters
        ----------
        json_filepath : Path
            The path to a JSON file containing the Imu fields

        Returns
        -------
        Imu
            Generated Imu message

        """
        imu_json = JSONConversion.load_from_json(
            json_filepath)

        imu = Imu()
        imu.header.frame_id = imu_json[
            'header']['frame_id']
        imu.header.stamp.sec = imu_json[
            'header']['stamp']['sec']
        imu.header.stamp.nanosec = imu_json[
            'header']['stamp']['nanosec']
        imu.orientation.x = imu_json['orientation']['x']
        imu.orientation.y = imu_json['orientation']['y']
        imu.orientation.z = imu_json['orientation']['z']
        imu.orientation.w = imu_json['orientation']['w']
        imu.angular_velocity.x = imu_json['angular_velocity']['x']
        imu.angular_velocity.y = imu_json['angular_velocity']['y']
        imu.angular_velocity.z = imu_json['angular_velocity']['z']
        imu.linear_acceleration.x = imu_json['linear_acceleration']['x']
        imu.linear_acceleration.y = imu_json['linear_acceleration']['y']
        imu.linear_acceleration.z = imu_json['linear_acceleration']['z']
        for i in range(9):
            imu.orientation_covariance[i] = imu_json['orientation_covariance'][i]
            imu.angular_velocity_covariance[i] = imu_json['angular_velocity_covariance'][i]
            imu.linear_acceleration_covariance[i] = imu_json['linear_acceleration_covariance'][i]
        return imu
