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

from typing import Union

import cuda.bindings.driver as driver
import cuda.bindings.runtime as runtime
from isaac_ros_nitros_bridge_interfaces.msg import (
    NitrosBridgeImage, NitrosBridgePointCloud, NitrosBridgeTensorList)
from isaac_ros_pynitros.pynitros_type_views.pynitros_image_view import PyNitrosImageView
from isaac_ros_pynitros.pynitros_type_views.pynitros_point_cloud_view import PyNitrosPointCloudView
from isaac_ros_pynitros.pynitros_type_views.pynitros_tensor_list_view import PyNitrosTensorListView
from isaac_ros_tensor_list_interfaces.msg import TensorList

from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2


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
        NitrosBridgeTensorList: PyNitrosTensorListView,
        NitrosBridgePointCloud: PyNitrosPointCloudView
    }

    BRIDGE_TYPE_TO_RAW_TYPE = {
        NitrosBridgeImage: Image,
        NitrosBridgeTensorList: TensorList,
        NitrosBridgePointCloud: PointCloud2
    }

    def __init__(
            self, node, message_type, sub_topic_name, enable_ros_subscribe=True,
            qos_profile: Union[QoSProfile, int] = 10):
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
        qos_profile : Union[QoSProfile, int]
            QoS profile to use for the subscriber.

        """
        self.node = node
        self.message_type = message_type
        self.sub_topic_name = sub_topic_name
        self.enable_ros_subscribe = enable_ros_subscribe
        self.qos_profile = qos_profile

        self.event = None
        self._cuda_memblock_fd_to_ptr = {}
        self._cuda_memblock_handle = []
        self._cuda_memblock_mapping = []

        # Initialize the CUDA driver API
        err, = driver.cuInit(0)
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
                self.listener_callback_ros, qos_profile=self.qos_profile)
        else:
            self.subscription = self.node.create_subscription(
                self.message_type,
                namespaced_sub_topic_name,
                self.listener_callback, qos_profile=self.qos_profile)

        self.input_callback = input_callback

    def listener_callback_ros(self, raw_msg):
        pynitros_view = self.BRIDGE_TYPE_TO_PYNITROS_VIEW[self.message_type](raw_msg)
        self.input_callback(pynitros_view)
        pynitros_view.postprocess()

    def listener_callback(self, nitros_bridge_msg):
        if nitros_bridge_msg.cuda_event_handle and not self.event:
            self.event_handle = runtime.cudaIpcEventHandle_t()
            self.event_handle.reserved = nitros_bridge_msg.cuda_event_handle

            err, self.event = runtime.cudaIpcOpenEventHandle(self.event_handle)
            self.ASSERT_CUDA_SUCCESS(err)

        # Synchronize to upstream Event
        if self.event:
            err, = runtime.cudaEventSynchronize(self.event)
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
            mapped_size = pynitros_view.get_mapped_size()
            self._cuda_memblock_fd_to_ptr[(sender_pid, memblock_fd)] = gpu_ptr
            self._cuda_memblock_handle.append(gpu_handle)
            self._cuda_memblock_mapping.append((gpu_ptr, mapped_size))
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
        if isinstance(err, driver.CUresult):
            if err != driver.CUresult.CUDA_SUCCESS:
                raise RuntimeError(
                    f'[Cuda Error: {err}], {driver.cuGetErrorString(err)}')
        elif isinstance(err, runtime.cudaError_t):
            if (err != 0):
                raise RuntimeError(f'CudaRT Error: {err}')
        else:
            raise RuntimeError('Unknown error type: {}'.format(err))

    def __del__(self):
        if self.event:
            err, = runtime.cudaEventDestroy(self.event)
            self.ASSERT_CUDA_SUCCESS(err)

        for gpu_ptr, mapped_size in self._cuda_memblock_mapping:
            err, = driver.cuMemUnmap(gpu_ptr, mapped_size)
            self.ASSERT_CUDA_SUCCESS(err)
            err, = driver.cuMemAddressFree(gpu_ptr, mapped_size)
            self.ASSERT_CUDA_SUCCESS(err)

        for gpu_handle in self._cuda_memblock_handle:
            err, = driver.cuMemRelease(gpu_handle)
            self.ASSERT_CUDA_SUCCESS(err)
