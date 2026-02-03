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

from isaac_ros_nitros_bridge_interfaces.msg import NitrosBridgePointCloud
from isaac_ros_pynitros.isaac_ros_pynitros_publisher import PyNitrosPublisher
from isaac_ros_pynitros.isaac_ros_pynitros_subscriber import PyNitrosSubscriber
from isaac_ros_pynitros.pynitros_type_builders.pynitros_point_cloud_builder import (
    PyNitrosPointCloudBuilder)

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header


class PyNITROSPointCloudForwardNode(Node):
    """A PyNITROS node that listens to point cloud messages and forwards them."""

    def __init__(self, name='pynitros_point_cloud_forward_node'):
        super().__init__(name)
        self.declare_parameter('enable_ros_subscribe', True)
        self.declare_parameter('enable_ros_publish', True)

        self.enable_ros_subscribe = (
            self.get_parameter('enable_ros_subscribe').get_parameter_value().bool_value)
        self.enable_ros_publish = (
            self.get_parameter('enable_ros_publish').get_parameter_value().bool_value)

        # Choose input topic based on subscribe mode
        input_topic = (
            'topic_forward_input_ros'
            if self.enable_ros_subscribe else
            'topic_forward_input'
        )
        self.pynitros_subscriber = PyNitrosSubscriber(
            self, NitrosBridgePointCloud, input_topic,
            enable_ros_subscribe=self.enable_ros_subscribe)

        self.pynitros_subscriber.create_subscription(self.listener_callback)

        self.pynitros_publisher = PyNitrosPublisher(
            self, NitrosBridgePointCloud, 'topic_forward_output',
            'topic_forward_output_ros')

        self.pynitros_point_cloud_builder = PyNitrosPointCloudBuilder()

    def listener_callback(self, pynitros_point_cloud_view):
        """Forward the received point cloud message."""
        # Create header inline
        header = Header()
        header.frame_id = pynitros_point_cloud_view.get_frame_id()
        header.stamp.sec = pynitros_point_cloud_view.get_timestamp_seconds()
        header.stamp.nanosec = pynitros_point_cloud_view.get_timestamp_nanoseconds()

        # Build the point cloud message with all fields read directly from input
        built_msgs = self.pynitros_point_cloud_builder.build(
            points_data=pynitros_point_cloud_view.get_points_data(),
            height=pynitros_point_cloud_view.get_height(),
            width=pynitros_point_cloud_view.get_width(),
            fields=pynitros_point_cloud_view.get_fields(),
            is_bigendian=pynitros_point_cloud_view.is_bigendian(),
            point_step=pynitros_point_cloud_view.get_point_step(),
            row_step=pynitros_point_cloud_view.get_row_step(),
            is_dense=pynitros_point_cloud_view.is_dense(),
            header=header,
            device_id=0,
            enable_ros_publish=self.enable_ros_publish
        )

        # Publish
        self.pynitros_publisher.publish(built_msgs)


def main(args=None):
    node = None
    try:
        rclpy.init(args=args)
        node = PyNITROSPointCloudForwardNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        # only shut down if context is active
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
