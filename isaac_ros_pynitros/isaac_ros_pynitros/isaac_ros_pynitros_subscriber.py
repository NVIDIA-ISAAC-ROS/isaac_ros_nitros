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

from cuda import cuda, cudart
from isaac_ros_nitros_bridge_interfaces.msg import NitrosBridgeImage, NitrosBridgeTensorList
from isaac_ros_pynitros.pynitros_type_views.pynitros_image_view import PyNitrosImageView
from isaac_ros_pynitros.pynitros_type_views.pynitros_tensor_list_view import PyNitrosTensorListView
from isaac_ros_tensor_list_interfaces.msg import TensorList
from sensor_msgs.msg import Image


class PyNitrosSubscriber():
    """
    The PyNITROS Subscriber listens to PyNITROS Publisher or the NITROS Bridge node.

    When a message arrives, an internal callback is triggered to convert the GPU data handle
    back into a valid GPU pointer. The internal callback will wrap the incoming message
    into a PyNITROS View and invoke the user-defined callback.
    If enable_ros_subscribe is set to True, the subscriber will exclusively subscribe to
    ROS messages. In this case, the internal callback will perform CPU->GPU data transfer
    to wrap the message into a PyNITROS View.
    """

    BRIDGE_TYPE_TO_PYNITROS_VIEW = {
        NitrosBridgeImage: PyNitrosImageView,
        NitrosBridgeTensorList: PyNitrosTensorListView
    }

    BRIDGE_TYPE_TO_RAW_TYPE = {
        NitrosBridgeImage: Image,
        NitrosBridgeTensorList: TensorList
    }

    def __init__(
            self, node, message_type, sub_topic_name, enable_ros_subscribe=True):
        """
        Initialize PyNitrosSubscriber.

        Parameters
        ----------
        node : rclpy.node.Node
            ROS2 node creates this subscriber.
        message_type : PyNitros message type
            NITROS Bridge message type to subscribe.
        sub_topic_name : str
            Topic name of the subscribed message.
        enable_ros_subscribe : bool
            If True, the subscriber will exclusively subscribe to ROS messages.

        """
        self.node = node
        self.message_type = message_type
        self.sub_topic_name = sub_topic_name
        self.enable_ros_subscribe = enable_ros_subscribe

        self.event = None
        self._cuda_memblock_fd_to_ptr = {}
        self._cuda_memblock_handle = []

        # Initialize the CUDA driver API
        err, = cuda.cuInit(0)
        self.ASSERT_CUDA_SUCCESS(err)

    def create_subscription(self, input_callback):
        namespaced_sub_topic_name = ''
        if (self.node.get_namespace() == '/'):
            namespaced_sub_topic_name = self.sub_topic_name
        else:
            namespaced_sub_topic_name = '/'.join(
                [self.node.get_namespace(), self.sub_topic_name.strip('/')])

        if self.enable_ros_subscribe:
            self.subscription = self.node.create_subscription(
                self.BRIDGE_TYPE_TO_RAW_TYPE[self.message_type],
                namespaced_sub_topic_name,
                self.listener_callback_ros, 10)
        else:
            self.subscription = self.node.create_subscription(
                self.message_type,
                namespaced_sub_topic_name,
                self.listener_callback, 10)

        self.input_callback = input_callback

    def listener_callback_ros(self, raw_msg):
        pynitros_view = self.BRIDGE_TYPE_TO_PYNITROS_VIEW[self.message_type](raw_msg)
        self.input_callback(pynitros_view)
        pynitros_view.postprocess()

    def listener_callback(self, nitros_bridge_msg):
        if nitros_bridge_msg.cuda_event_handle and not self.event:
            self.event_handle = cudart.cudaIpcEventHandle_t()
            self.event_handle.reserved = nitros_bridge_msg.cuda_event_handle

            err, self.event = cudart.cudaIpcOpenEventHandle(self.event_handle)
            self.ASSERT_CUDA_SUCCESS(err)

        # Synchronize to upstream Event
        if self.event:
            err, = cudart.cudaEventSynchronize(self.event)
            self.ASSERT_CUDA_SUCCESS(err)

        sender_pid, memblock_fd = \
            self.BRIDGE_TYPE_TO_PYNITROS_VIEW[self.message_type].get_pid_fd(nitros_bridge_msg)
        if (sender_pid, memblock_fd) not in self._cuda_memblock_fd_to_ptr:
            # Get PyNITROS view from bridge message
            try:
                pynitros_view = self.BRIDGE_TYPE_TO_PYNITROS_VIEW[
                    self.message_type](nitros_bridge_msg)
            except ValueError:
                self.node.get_logger().warn('Failed to create PyNITROS view from bridge msg')
                return
            gpu_ptr = pynitros_view.get_buffer()
            gpu_handle = pynitros_view.get_handle()
            self._cuda_memblock_fd_to_ptr[(sender_pid, memblock_fd)] = gpu_ptr
            self._cuda_memblock_handle.append(gpu_handle)
        else:
            gpu_ptr = self._cuda_memblock_fd_to_ptr[(sender_pid, memblock_fd)]
            try:
                pynitros_view = self.BRIDGE_TYPE_TO_PYNITROS_VIEW[
                    self.message_type](nitros_bridge_msg, gpu_ptr)
            except ValueError:
                self.node.get_logger().warn('Failed to create PyNITROS view from bridge msg')
                return

        # Invoke user-defined callback
        self.input_callback(pynitros_view)

        # Decrease Refcount
        pynitros_view.postprocess()

    def ASSERT_CUDA_SUCCESS(self, err):
        if isinstance(err, cuda.CUresult):
            if err != cuda.CUresult.CUDA_SUCCESS:
                raise RuntimeError(
                    f'[Cuda Error: {err}], {cuda.cuGetErrorString(err)}')
        elif isinstance(err, cudart.cudaError_t):
            if (err != 0):
                raise RuntimeError(f'CudaRT Error: {err}')
        else:
            raise RuntimeError('Unknown error type: {}'.format(err))

    def __del__(self):
        for gpu_handle in self._cuda_memblock_handle:
            err, = cuda.cuMemRelease(gpu_handle)
            self.ASSERT_CUDA_SUCCESS(err)
