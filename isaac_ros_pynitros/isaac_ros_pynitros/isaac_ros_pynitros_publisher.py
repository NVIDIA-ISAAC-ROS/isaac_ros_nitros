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
from isaac_ros_nitros_bridge_interfaces.msg import NitrosBridgeTensorList
from isaac_ros_tensor_list_interfaces.msg import TensorList
from sensor_msgs.msg import Image


class PyNitrosPublisher():
    """Publish built messages from PyNITROS Builder."""

    def __init__(self, node, message_type, pub_topic, pub_topic_raw):
        """
        Initialize PyNitrosPublisher.

        Parameters
        ----------
        node : rclpy.node.Node
            ROS2 node creates this publisher.
        message_type : PyNitros message type
            NITROS Bridge message type to publish.
        pub_topic : str
            ROS2 topic name of NITROS bridge message.
        pub_topic_raw : str
            ROS2 topic name of raw ROS message.

        """
        self.node = node
        self.msg_map = {NitrosBridgeImage: Image,
                        NitrosBridgeTensorList: TensorList}

        # Regular PyNITROS Pub
        self.message_type = message_type
        self.pub_topic = pub_topic

        # If enable_ros_publish == True
        self.pub_topic_raw = pub_topic_raw

        # Setup both publishers
        self.publisher_ = self.node.create_publisher(
            self.message_type, self.pub_topic, 10)
        self.publisher_raw_ = self.node.create_publisher(
            self.msg_map[self.message_type], self.pub_topic_raw, 10)

    def publish(self, pynitros_built_msg):
        """Publish PyNITROS built messages."""
        bridge_msg = pynitros_built_msg[0]
        msg_raw = pynitros_built_msg[1]

        # Publish PyNITROS bridge msg
        self.publisher_.publish(bridge_msg)

        # Publish raw image if enable_ros_publish is true
        if msg_raw:
            self.publisher_raw_.publish(msg_raw)
