# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from isaac_ros_nitros_bridge_interfaces.msg import NitrosBridgeImage
from isaac_ros_pynitros.isaac_ros_pynitros_message_filter import PyNitrosMessageFilter
from isaac_ros_pynitros.isaac_ros_pynitros_publisher import PyNitrosPublisher
from isaac_ros_pynitros.isaac_ros_pynitros_subscriber import PyNitrosSubscriber
from isaac_ros_pynitros.pynitros_type_builders.pynitros_image_builder import PyNitrosImageBuilder
from message_filters import Subscriber, TimeSynchronizer

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header


class PyNITROSMessageFilterSyncNode(Node):
    """A PyNITROS node listen to synced image and camera info."""

    def __init__(self, name='pynitros_message_filter_sync_node'):
        super().__init__(name)
        self.declare_parameter('enable_ros_subscribe', False)
        self.declare_parameter('enable_ros_publish', False)

        self.enable_ros_subscribe = \
            self.get_parameter('enable_ros_subscribe').get_parameter_value().bool_value
        self.enable_ros_publish = \
            self.get_parameter('enable_ros_publish').get_parameter_value().bool_value

        self.pynitros_subscriber = PyNitrosSubscriber(
            self, NitrosBridgeImage, 'image',
            enable_ros_subscribe=self.enable_ros_subscribe)
        self.camera_info_sub = Subscriber(self, CameraInfo, 'camera_info')

        self.message_filter = PyNitrosMessageFilter(self,
                                                    [self.pynitros_subscriber,
                                                     self.camera_info_sub],
                                                    TimeSynchronizer,
                                                    self.listener_callback,
                                                    queue_size=1)

        self.pynitros_publisher = PyNitrosPublisher(
            self, NitrosBridgeImage, 'synced_output', 'synced_output_ros')

        self.pynitros_image_builder = PyNitrosImageBuilder(
            num_buffer=40, timeout=5)

    def listener_callback(self, pynitros_image_view, camera_info):
        header = Header()
        header.frame_id = pynitros_image_view.get_frame_id()
        header.stamp.sec = pynitros_image_view.get_timestamp_seconds()
        header.stamp.nanosec = pynitros_image_view.get_timestamp_nanoseconds()
        # Check if image and camera info are in sync
        if (header.stamp == camera_info.header.stamp):
            self.get_logger().info('Image and Camera Info are in sync')
        else:
            self.get_logger().error('Image and Camera Info are not in sync')
        # Build Image
        built_image = self.pynitros_image_builder.build(pynitros_image_view.get_buffer(),
                                                        pynitros_image_view.get_height(),
                                                        pynitros_image_view.get_width(),
                                                        pynitros_image_view.get_stride(),
                                                        pynitros_image_view.get_encoding(),
                                                        header,
                                                        0,
                                                        self.enable_ros_publish)

        # Publish
        self.pynitros_publisher.publish(built_image)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = PyNITROSMessageFilterSyncNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # only shut down if context is active
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
