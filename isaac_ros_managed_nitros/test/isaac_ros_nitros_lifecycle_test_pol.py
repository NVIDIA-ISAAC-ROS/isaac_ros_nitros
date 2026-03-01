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

The test loads NitrosEmptyLifecycleNode in a component container and drives it through
the full lifecycle state machine:

  configure → activate → deactivate → cleanup → configure → shutdown

This exercises every on_*() callback and verifies that ManagedNitrosPublisher and
ManagedNitrosSubscriber are constructed, used, and destroyed correctly at each stage.
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
    """Test the full lifecycle state machine of NitrosEmptyLifecycleNode."""

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

    def test_lifecycle_full_sequence(self):
        """
        Drive the complete lifecycle state machine in one ordered test.

        A single test method avoids cross-test state leakage (all test methods share
        the same launched node; leaving the node in a non-unconfigured state would
        make subsequent transitions invalid).

        Sequence:
          1. configure   (unconfigured → inactive): constructs ManagedNitrosPublisher
             and ManagedNitrosSubscriber
          2. activate    (inactive → active):       exercises on_activate()
          3. deactivate  (active → inactive):       exercises on_deactivate()
          4. cleanup     (inactive → unconfigured): destroys pub/sub, verifies
             on_cleanup() returns SUCCESS
          5. configure   (unconfigured → inactive): re-constructs pub/sub, verifies
             resources can be re-created after a cleanup
          6. shutdown    (inactive → finalized):    destroys pub/sub via on_shutdown(),
             verifies clean teardown from the inactive state
        """
        client = self._make_change_state_client()

        start = time.time()
        self.assertTrue(
            client.wait_for_service(timeout_sec=10.0),
            msg=f'ChangeState service not available after {time.time() - start:.1f}s',
        )

        # Step 1: configure — creates publisher + subscriber
        result = self._send_transition(client, Transition.TRANSITION_CONFIGURE)
        self.assertIsNotNone(result, 'configure (1st) future returned None')
        self.assertTrue(result.success, 'configure (1st) transition failed')

        # Step 2: activate — exercises on_activate()
        result = self._send_transition(client, Transition.TRANSITION_ACTIVATE)
        self.assertIsNotNone(result, 'activate future returned None')
        self.assertTrue(result.success, 'activate transition failed')

        # Step 3: deactivate — exercises on_deactivate()
        result = self._send_transition(client, Transition.TRANSITION_DEACTIVATE)
        self.assertIsNotNone(result, 'deactivate future returned None')
        self.assertTrue(result.success, 'deactivate transition failed')

        # Step 4: cleanup — destroys publisher + subscriber
        result = self._send_transition(client, Transition.TRANSITION_CLEANUP)
        self.assertIsNotNone(result, 'cleanup future returned None')
        self.assertTrue(result.success, 'cleanup transition failed')

        # Step 5: configure again — verifies resources can be re-created after cleanup
        result = self._send_transition(client, Transition.TRANSITION_CONFIGURE)
        self.assertIsNotNone(result, 'configure (2nd) future returned None')
        self.assertTrue(result.success, 'configure (2nd) transition failed')

        # Step 6: shutdown — destroys pub/sub via on_shutdown(), node enters finalized
        result = self._send_transition(client, Transition.TRANSITION_INACTIVE_SHUTDOWN)
        self.assertIsNotNone(result, 'shutdown future returned None')
        self.assertTrue(result.success, 'shutdown transition failed')
