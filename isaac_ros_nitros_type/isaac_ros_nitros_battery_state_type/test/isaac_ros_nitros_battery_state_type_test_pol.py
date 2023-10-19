# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Proof-of-Life test for the NitrosBatteryState type adapter."""

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

from sensor_msgs.msg import BatteryState


@pytest.mark.rostest
def generate_test_description():
    """Generate launch description with all ROS 2 nodes for testing."""
    test_ns = IsaacROSNitrosBatteryStateTest.generate_namespace()
    container = ComposableNodeContainer(
        name='test_container',
        namespace='isaac_ros_nitros_container',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_nitros_battery_state_type',
                plugin='nvidia::isaac_ros::nitros::NitrosBatteryStateForwardNode',
                name='NitrosBatteryStateForwardNode',
                namespace=test_ns,
                parameters=[{
                    'compatible_format': 'nitros_battery_state'
                }],
                remappings=[
                    (test_ns+'/topic_forward_input', test_ns+'/input'),
                    (test_ns+'/topic_forward_output', test_ns+'/output'),
                ]
            ),
        ],
        output='both',
        arguments=['--ros-args', '--log-level', 'NitrosBatteryState:=debug'],
    )

    return IsaacROSNitrosBatteryStateTest.generate_test_description([
        container,
        launch.actions.TimerAction(
            period=2.5, actions=[launch_testing.actions.ReadyToTest()])
    ])


class IsaacROSNitrosBatteryStateTest(IsaacROSBaseTest):
    """Validate NitrosBatteryState type adapter."""

    filepath = pathlib.Path(os.path.dirname(__file__))

    @IsaacROSBaseTest.for_each_test_case(subfolder='nitros_battery_state')
    def test_nitros_battery_state_type_conversions(self, test_folder) -> None:
        """Expect the message from NitrosBatteryState type to be compatible with source."""
        self.generate_namespace_lookup(['input', 'output'])
        received_messages = {}

        received_message_sub = self.create_logging_subscribers(
            subscription_requests=[('output', BatteryState)],
            received_messages=received_messages
        )

        battery_state_pub = self.node.create_publisher(
            BatteryState, self.namespaces['input'], self.DEFAULT_QOS)

        try:
            battery_state = self.load_battery_state_from_json(test_folder / 'battery_state.json')

            # Wait at most TIMEOUT seconds for subscriber to respond
            TIMEOUT = 10
            end_time = time.time() + TIMEOUT

            done = False
            while time.time() < end_time:
                timestamp = self.node.get_clock().now().to_msg()
                battery_state.header.stamp = timestamp

                battery_state_pub.publish(battery_state)

                rclpy.spin_once(self.node, timeout_sec=0.1)

                # If we have received a message on the output topic, break
                if 'output' in received_messages:
                    done = True
                    break

            self.assertTrue(done, "Didn't receive output on the output topic!")

            received_battery_state = received_messages['output']

            # Only test for data passed through gxf
            # Header
            self.assertEqual(battery_state.header.stamp.sec,
                             received_battery_state.header.stamp.sec)
            self.assertEqual(battery_state.header.stamp.nanosec,
                             received_battery_state.header.stamp.nanosec)
            self.assertEqual(battery_state.header.frame_id,
                             received_battery_state.header.frame_id)
            # Values
            self.assertEqual(battery_state.voltage,
                             round(received_battery_state.voltage, 1))
            self.assertEqual(battery_state.percentage,
                             round(received_battery_state.percentage, 1))
            self.assertEqual(battery_state.power_supply_status,
                             received_battery_state.power_supply_status)
            print('The received battery_state message has been verified successfully')
        finally:
            self.node.destroy_subscription(received_message_sub)
            self.node.destroy_publisher(battery_state_pub)

    @staticmethod
    def load_battery_state_from_json(
            json_filepath: pathlib.Path) -> BatteryState:
        """
        Load a BatteryState message from a JSON filepath.

        Parameters
        ----------
        json_filepath : Path
            The path to a JSON file containing the BatteryState fields

        Returns
        -------
        BatteryState
            Generated BatteryState message

        """
        battery_state_json = JSONConversion.load_from_json(
            json_filepath)

        battery_state = BatteryState()
        battery_state.header.frame_id = battery_state_json[
            'header']['frame_id']
        battery_state.header.stamp.sec = battery_state_json[
            'header']['stamp']['sec']
        battery_state.header.stamp.nanosec = battery_state_json[
            'header']['stamp']['nanosec']
        battery_state.voltage = battery_state_json['voltage']
        battery_state.temperature = battery_state_json['temperature']
        battery_state.current = battery_state_json['current']
        battery_state.charge = battery_state_json['charge']
        battery_state.capacity = battery_state_json['capacity']
        battery_state.design_capacity = battery_state_json['design_capacity']
        battery_state.percentage = battery_state_json['percentage']
        battery_state.power_supply_status = battery_state_json['power_supply_status']
        battery_state.power_supply_health = battery_state_json['power_supply_health']
        battery_state.power_supply_technology = battery_state_json['power_supply_technology']
        battery_state.present = battery_state_json['present']
        for i in range(len(battery_state_json['cell_voltage'])):
            battery_state.cell_voltage.append(battery_state_json['cell_voltage'][i])
        for i in range(len(battery_state_json['cell_temperature'])):
            battery_state.cell_temperature.append(battery_state_json['cell_temperature'][i])
        battery_state.location = battery_state_json['location']
        battery_state.serial_number = battery_state_json['serial_number']
        return battery_state
