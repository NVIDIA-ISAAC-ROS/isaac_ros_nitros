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

"""
Launch test verifying that ManagedNitrosPublisher/Subscriber work inside a LifecycleNode.

The test loads NitrosEmptyLifecycleNode in a component container, drives it through
the configure transition (which creates both a ManagedNitrosPublisher and a
ManagedNitrosSubscriber), and verifies the node transitions succeed.
"""

import time

from isaac_ros_test import IsaacROSBaseTest
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import launch_testing
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
import pytest
import rclpy


@pytest.mark.rostest
def generate_test_description():
    """Generate launch description with NitrosEmptyLifecycleNode."""
    container = ComposableNodeContainer(
        name='nitros_lifecycle_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_managed_nitros',
                plugin='nvidia::isaac_ros::nitros::NitrosEmptyLifecycleNode',
                name='nitros_empty_lifecycle_node',
            ),
        ],
        output='both',
    )

    return TestNitrosLifecycle.generate_test_description([
        container,
        launch.actions.TimerAction(
            period=2.5, actions=[launch_testing.actions.ReadyToTest()])
    ])


class TestNitrosLifecycle(IsaacROSBaseTest):
    """Test that NitrosEmptyLifecycleNode pub/sub construction works via lifecycle."""

    package = 'isaac_ros_managed_nitros'

    def _make_change_state_client(self):
        """Return a ChangeState client for the lifecycle node."""
        return self.node.create_client(
            ChangeState,
            '/nitros_empty_lifecycle_node/change_state',
        )

    def _send_transition(self, client, transition_id, timeout_sec=10.0):
        """Send a lifecycle transition and return the result, or None on timeout."""
        available = client.wait_for_service(timeout_sec=timeout_sec)
        if not available:
            return None
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
        return future.result()

    def test_lifecycle_configure_and_cleanup(self):
        """
        Drive configure → cleanup → configure cycle in one test.

        A single test method avoids cross-test state leakage: both test methods
        would share the same launched lifecycle node, so a standalone
        'test_configure' would leave the node in 'inactive', making a subsequent
        'configure' call invalid (configure is only valid from 'unconfigured').

        Sequence:
          1. configure  (unconfigured → inactive): constructs ManagedNitrosPublisher
             and ManagedNitrosSubscriber via LifecycleNode
          2. cleanup    (inactive → unconfigured): destroys pub/sub, verifies
             on_cleanup() returns SUCCESS
          3. configure  (unconfigured → inactive): re-constructs pub/sub, verifies
             resources can be re-created after a cleanup
        """
        client = self._make_change_state_client()

        start = time.time()
        self.assertTrue(
            client.wait_for_service(timeout_sec=10.0),
            msg=f'ChangeState service not available after {time.time() - start:.1f}s',
        )

        # Step 1: configure — creates publisher + subscriber
        configure_result = self._send_transition(client, Transition.TRANSITION_CONFIGURE)
        self.assertIsNotNone(configure_result, 'configure (1st) future returned None')
        self.assertTrue(configure_result.success, 'configure (1st) transition failed')

        # Step 2: cleanup — destroys publisher + subscriber
        cleanup_result = self._send_transition(client, Transition.TRANSITION_CLEANUP)
        self.assertIsNotNone(cleanup_result, 'cleanup future returned None')
        self.assertTrue(cleanup_result.success, 'cleanup transition failed')

        # Step 3: configure again — verifies resources can be re-created
        reconfigure_result = self._send_transition(client, Transition.TRANSITION_CONFIGURE)
        self.assertIsNotNone(reconfigure_result, 'configure (2nd) future returned None')
        self.assertTrue(reconfigure_result.success, 'configure (2nd) transition failed')
