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

from isaac_ros_nitros_bridge_interfaces.msg import NitrosBridgeTensorList
from isaac_ros_pynitros.isaac_ros_pynitros_publisher import PyNitrosPublisher
from isaac_ros_pynitros.isaac_ros_pynitros_subscriber import PyNitrosSubscriber
from isaac_ros_pynitros.pynitros_type_builders.pynitros_tensor_list_builder \
    import PyNitrosTensorListBuilder

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header


class PyNITROSTensorListForwardNode(Node):
    """A PyNITROS node listens to tensor list message and publish it again."""

    def __init__(self, name='pynitros_tensor_list_forward_node'):
        super().__init__(name)
        self.declare_parameter('enable_ros_subscribe', False)
        self.declare_parameter('enable_ros_publish', False)

        self.enable_ros_subscribe = \
            self.get_parameter('enable_ros_subscribe').get_parameter_value().bool_value
        self.enable_ros_publish = \
            self.get_parameter('enable_ros_publish').get_parameter_value().bool_value

        self.pynitros_subscriber = PyNitrosSubscriber(
            self, NitrosBridgeTensorList, 'pynitros_input_msg',
            enable_ros_subscribe=self.enable_ros_subscribe)
        self.pynitros_subscriber.create_subscription(self.listener_callback)

        self.pynitros_publisher = PyNitrosPublisher(
            self, NitrosBridgeTensorList, 'pynitros_output_msg', 'pynitros_output_msg_ros')

        self.pynitros_tensor_list_builder = PyNitrosTensorListBuilder(num_buffer=20)

    def listener_callback(self, pynitros_tensor_list_view):
        tensor_list_view = pynitros_tensor_list_view.get_all_tensors()
        tensor_list = []

        # Build Tensors
        for tensor_view in tensor_list_view:
            tensor = self.pynitros_tensor_list_builder.build_tensor(
                tensor_view.get_buffer(),
                tensor_view.get_name(),
                tensor_view.get_shape(),
                tensor_view.get_element_type())
            tensor_list.append(tensor)

        # Build TensorList
        header = Header()
        header.frame_id = pynitros_tensor_list_view.get_frame_id()
        header.stamp.sec = pynitros_tensor_list_view.get_timestamp_seconds()
        header.stamp.nanosec = pynitros_tensor_list_view.get_timestamp_nanoseconds()
        pynitros_tensors = self.pynitros_tensor_list_builder.build(tensor_list,
                                                                   header,
                                                                   0,
                                                                   self.enable_ros_publish)

        # Publish
        self.pynitros_publisher.publish(pynitros_tensors)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = PyNITROSTensorListForwardNode()
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
