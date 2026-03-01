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
Launch test verifying that ManagedNitrosPublisher works inside a LifecycleNode.

The test loads NitrosEmptyLifecycleNode in a component container, drives it
through the configure transition, and verifies the node reaches the 'inactive'
state without errors.
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
        output='screen',
    )

    return generate_test_description.generate(
        container,
        launch_testing.actions.ReadyToTest(),
    )


generate_test_description.generate = lambda *actions: launch.LaunchDescription(list(actions))


class TestNitrosLifecycle(IsaacROSBaseTest):
    """Test that NitrosEmptyLifecycleNode transitions to 'inactive' state."""

    package = 'isaac_ros_managed_nitros'

    def test_lifecycle_configure(self):
        """Drive the lifecycle node through configure and verify success."""
        self.node.create_client(
            ChangeState,
            '/nitros_empty_lifecycle_node/change_state',
        )
        change_state_client = self.node.create_client(
            ChangeState,
            '/nitros_empty_lifecycle_node/change_state',
        )

        # Wait for the service to be available
        start = time.time()
        available = change_state_client.wait_for_service(timeout_sec=10.0)
        self.assertTrue(
            available,
            msg=f'ChangeState service not available after {time.time() - start:.1f}s',
        )

        # Send configure transition
        request = ChangeState.Request()
        request.transition.id = Transition.TRANSITION_CONFIGURE
        future = change_state_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)

        self.assertIsNotNone(future.result(), 'ChangeState future returned None')
        self.assertTrue(future.result().success, 'configure transition failed')
