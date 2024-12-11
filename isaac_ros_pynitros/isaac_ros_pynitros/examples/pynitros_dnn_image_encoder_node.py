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

from isaac_ros_nitros_bridge_interfaces.msg import NitrosBridgeImage, NitrosBridgeTensorList
from isaac_ros_pynitros.isaac_ros_pynitros_publisher import PyNitrosPublisher
from isaac_ros_pynitros.isaac_ros_pynitros_subscriber import PyNitrosSubscriber
from isaac_ros_pynitros.pynitros_type_builders.pynitros_tensor_list_builder \
    import PyNitrosTensorListBuilder


import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

import torch
import torchvision.transforms as T


class PyNITROSDNNImageEncoderNode(Node):
    """A PyNITROS node listens to image message and publish it again."""

    def __init__(self, name='pynitros_image_forward_node'):
        super().__init__(name)
        self.declare_parameter('enable_ros_subscribe', False)
        self.declare_parameter('enable_ros_publish', False)
        self.declare_parameter('network_image_width', 960)
        self.declare_parameter('network_image_height', 544)
        self.declare_parameter('image_mean', [0.5, 0.5, 0.5])
        self.declare_parameter('image_std', [0.5, 0.5, 0.5])

        self.enable_ros_subscribe = \
            self.get_parameter('enable_ros_subscribe').get_parameter_value().bool_value
        self.enable_ros_publish = \
            self.get_parameter('enable_ros_publish').get_parameter_value().bool_value

        self.network_image_width = \
            self.get_parameter('network_image_width').get_parameter_value().integer_value
        self.network_image_height = \
            self.get_parameter('network_image_height').get_parameter_value().integer_value

        self.image_mean = \
            self.get_parameter('image_mean').get_parameter_value().double_array_value
        self.image_std = \
            self.get_parameter('image_std').get_parameter_value().double_array_value

        self.pynitros_subscriber = PyNitrosSubscriber(
            self, NitrosBridgeImage, 'pynitros_input_msg',
            enable_ros_subscribe=self.enable_ros_subscribe)
        self.pynitros_subscriber.create_subscription(self.listener_callback)

        self.pynitros_publisher = PyNitrosPublisher(
            self, NitrosBridgeTensorList, 'pynitros_output_msg', 'pynitros_output_msg_ros')

        self.pynitros_tensor_list_builder = PyNitrosTensorListBuilder(
            num_buffer=40, timeout=5)

    def listener_callback(self, pynitros_image_view):
        header = Header()
        header.frame_id = pynitros_image_view.get_frame_id()
        header.stamp.sec = pynitros_image_view.get_timestamp_seconds()
        header.stamp.nanosec = pynitros_image_view.get_timestamp_nanoseconds()

        tensor_image = torch.as_tensor(pynitros_image_view, device='cuda', dtype=torch.uint8)
        reshaped_image = tensor_image.reshape(pynitros_image_view.get_height(),
                                              pynitros_image_view.get_width(),
                                              3)

        # Switch from interleaved to planar
        planar_tensor = reshaped_image.permute(2, 0, 1)

        transforms = T.Compose([
            T.Resize((self.network_image_height, self.network_image_width)),
            T.ConvertImageDtype(torch.float),
            T.Normalize(self.image_mean, self.image_std)
        ]
        )
        norm_tensor = transforms(planar_tensor)
        out_tensor = norm_tensor.contiguous()

        tensor_list = []
        pynitros_tensor = self.pynitros_tensor_list_builder.build_tensor(
            out_tensor.data_ptr(),
            'input_tensor',
            out_tensor.shape,
            9)
        tensor_list.append(pynitros_tensor)

        # # Build TensorList
        pynitros_tensor_list = self.pynitros_tensor_list_builder.build(tensor_list,
                                                                       header,
                                                                       0,
                                                                       self.enable_ros_publish)

        self.pynitros_publisher.publish(pynitros_tensor_list)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = PyNITROSDNNImageEncoderNode()
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
