# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import time

from isaac_ros_tensor_list_interfaces.msg import Tensor, TensorList, TensorShape
from isaac_ros_test import IsaacROSBaseTest

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import launch_testing
import numpy as np
import pytest
import rclpy


@pytest.mark.rostest
def generate_test_description():
    """Generate launch description with NitrosNode test node."""
    test_ns = IsaacROSNitrosTensorListTest.generate_namespace()
    container = ComposableNodeContainer(
        name='image_container',
        namespace='isaac_ros_nitros_container',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_nitros_tensor_list_type',
                plugin='nvidia::isaac_ros::nitros::NitrosTensorListForwardNode',
                name='NitrosTensorListForwardNode',
                namespace=test_ns,
                parameters=[{
                    'compatible_format': 'nitros_tensor_list_nchw'
                }]
            ),
        ],
        output='both',
        arguments=['--ros-args', '--log-level', 'info'],
    )

    return IsaacROSNitrosTensorListTest.generate_test_description([
        container,
        launch.actions.TimerAction(
            period=2.5, actions=[launch_testing.actions.ReadyToTest()])
    ])


class IsaacROSNitrosTensorListTest(IsaacROSBaseTest):
    """
    Proof-of-Life Test for Isaac ROS Nitros Node.

    1. Sets up ROS publisher to send TensorList values
    2. Sets up ROS subscriber to listen to output channel of NitrosNode
    3. Verify received messages
    """

    def test_forward_node(self) -> None:
        self.node._logger.info('Starting Isaac ROS NitrosNode POL Test')

        # Subscriber
        received_messages = {}

        subscriber_topic_namespace = self.generate_namespace('topic_forward_output')
        test_subscribers = [
            (subscriber_topic_namespace, TensorList)
        ]

        subs = self.create_logging_subscribers(
            subscription_requests=test_subscribers,
            received_messages=received_messages,
            use_namespace_lookup=False,
            accept_multiple_messages=True,
            add_received_message_timestamps=True
        )

        # Publisher
        publisher_topic_namespace = self.generate_namespace('topic_forward_input')
        pub = self.node.create_publisher(
            TensorList,
            publisher_topic_namespace,
            self.DEFAULT_QOS)

        try:
            # Construct test tensor list
            DATA_TYPE = 9
            INPUT_TENSOR_DIMENSIONS = [1, 3, 100, 100]
            INPUT_TENSOR_NAME = 'input'
            INPUT_TENSOR_STRIDE = 4

            tensor_list = TensorList()
            tensor = Tensor()
            tensor_shape = TensorShape()

            tensor_shape.rank = len(INPUT_TENSOR_DIMENSIONS)
            tensor_shape.dims = INPUT_TENSOR_DIMENSIONS

            tensor.shape = tensor_shape
            tensor.name = INPUT_TENSOR_NAME
            tensor.data_type = DATA_TYPE
            tensor.strides = []

            data_length = INPUT_TENSOR_STRIDE * np.prod(INPUT_TENSOR_DIMENSIONS)
            tensor.data = np.random.randint(256, size=data_length).tolist()

            tensor_list.tensors = [tensor]

            # Start sending messages
            self.node.get_logger().info('Start publishing messages')
            sent_count = 0
            end_time = time.time() + 2.0
            while time.time() < end_time:
                sent_count += 1
                pub.publish(tensor_list)
                rclpy.spin_once(self.node, timeout_sec=0.2)

            # Conclude the test
            received_count = len(received_messages[subscriber_topic_namespace])
            self.node._logger.info(
                f'Test Results:\n'
                f'# of Messages Sent: {sent_count}\n'
                f'# of Messages Received: {received_count}\n'
                f'# of Messages Dropped: {sent_count - received_count}\n'
                f'Message Drop Rate: {((sent_count-received_count)/sent_count)*100}%'
            )

            self.assertGreater(len(received_messages[subscriber_topic_namespace]), 0)
            for i in range(data_length):
                self.assertEqual(
                    received_messages[subscriber_topic_namespace][-1][0].tensors[0].data[i],
                    tensor.data[i])

            self.node._logger.info('Source and received tensor lists are matched.')

        finally:
            [self.node.destroy_subscription(sub) for sub in subs]
            self.assertTrue(self.node.destroy_publisher(pub))
