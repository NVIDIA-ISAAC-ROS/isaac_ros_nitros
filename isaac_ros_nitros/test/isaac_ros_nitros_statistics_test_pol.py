# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import time

from isaac_ros_nitros_interfaces.msg import TopicStatistics
from isaac_ros_test import IsaacROSBaseTest
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import launch_testing
import pytest
import rclpy
from std_msgs.msg import Empty


@pytest.mark.rostest
def generate_test_description():
    """Generate launch description with NitrosNode test node."""
    test_ns = IsaacROSNitrosNodeTest.generate_namespace()
    container = ComposableNodeContainer(
        name='image_container',
        namespace='isaac_ros_nitros_container',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_nitros',
                plugin='nvidia::isaac_ros::nitros::NitrosNode',
                name='isaac_ros_nitros',
                namespace=test_ns,
                parameters=[{
                        'compatible_format': 'nitros_empty',
                        'enable_statistics': True,
                        'statistics_publish_rate': 2.0,
                        'filter_window_size': 7
                }]
            ),
            ComposableNode(
                package='isaac_ros_nitros',
                plugin='nvidia::isaac_ros::nitros::NitrosNode',
                name='isaac_ros_nitros',
                namespace=test_ns+'/mid1',
                parameters=[{
                        'compatible_format': 'nitros_empty',
                        'enable_statistics': True,
                        'statistics_publish_rate': 3.0,
                        'filter_window_size': 12
                }],
                remappings=[
                    (test_ns+'/mid1/topic_forward_input',
                     test_ns+'/topic_forward_output'),
                    (test_ns+'/mid1/topic_forward_output',
                     test_ns+'/final/topic_forward_output'),
                ]
            )
        ],
        output='both',
        arguments=['--ros-args', '--log-level', 'info'],
    )

    return IsaacROSNitrosNodeTest.generate_test_description([
        container,
        launch.actions.TimerAction(
            period=2.5, actions=[launch_testing.actions.ReadyToTest()])
    ])


class IsaacROSNitrosNodeTest(IsaacROSBaseTest):
    """
    Proof-of-Life Test for Isaac ROS Nitros Node.

    1. Sets up ROS publisher to send Empty values
    2. Sets up ROS subscriber to listen to output channel of NitrosNode
    3. Verify received messages
    """

    def test_forward_node(self) -> None:
        self.node._logger.info('Starting Isaac ROS NitrosNode POL Test')

        # Subscriber
        received_messages = {}

        test_subscribers = [
            ('/nitros_statistics', TopicStatistics)
        ]

        subs = self.create_logging_subscribers(
            subscription_requests=test_subscribers,
            received_messages=received_messages,
            use_namespace_lookup=False,
            accept_multiple_messages=True
        )

        # Publisher
        publisher_topic_namespace = self.generate_namespace('topic_forward_input')
        pub = self.node.create_publisher(
            Empty,
            publisher_topic_namespace,
            self.DEFAULT_QOS)

        try:
            # Construct test message
            msg = Empty()

            # Testing frame rate
            test_frame_rate = 5

            # Start sending messages
            self.node.get_logger().info('Start publishing messages')

            # Publish messages at a fixed frame rate set by test_frame_rate
            end_time = time.time() + 2
            sent_count = 0
            while time.time() < end_time:
                sent_count += 1
                pub.publish(msg)
                time.sleep(1/test_frame_rate)

            # Run the queued ROS subscriber callbacks
            rclpy.spin_once(self.node, timeout_sec=0.1)

            # Assert that we have received atleast one message on the output topic
            self.assertNotEqual(
                len(received_messages['/nitros_statistics']),
                0)

            if len(received_messages['/nitros_statistics']) > 0:
                # Assert that the measured frame rate within the acceptable error window
                self.assertAlmostEqual(received_messages['/nitros_statistics'][0].frame_rate,
                                       test_frame_rate, None,
                                       'measured frame rate error is too high', 0.1)

        finally:
            [self.node.destroy_subscription(sub) for sub in subs]
            self.assertTrue(self.node.destroy_publisher(pub))
