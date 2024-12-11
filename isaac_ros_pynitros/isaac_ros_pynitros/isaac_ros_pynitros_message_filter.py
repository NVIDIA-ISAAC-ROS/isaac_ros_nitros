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
from isaac_ros_pynitros.isaac_ros_pynitros_subscriber import PyNitrosSubscriber
from isaac_ros_pynitros.pynitros_type_views.pynitros_type_view_base import PyNitrosTypeViewBase

from message_filters import ApproximateTimeSynchronizer, Subscriber, TimeSynchronizer


class PyNitrosMessageFilter():
    """The PyNITROS Message filter synchronize to multiple subscribed topics."""

    def __init__(self, node, subscribers, synchronizer, callback, queue_size=10, slop=0.1):
        """
        Initialize PyNitrosSubscriber.

        Parameters
        ----------
        node : rclpy.node.Node
            ROS2 node creates this message filter.
        subscribers : list of PyNitrosSubscriber or rclpy.subscription.Subscriber
            List of subscribers to synchronize.
        synchronizer : TimeSynchronizer or ApproximateTimeSynchronizer
            Synchronizer type to use.
        callback : function
            User-defined callback function to process synchronized messages.
        queue_size : int
            Queue size of the synchronizer.
        slop : float
            Slop of the ApproximateTimeSynchronizer.

        """
        rclpy_subs = []
        self.subscribers = subscribers
        self.node = node
        for sub in subscribers:
            if (isinstance(sub, PyNitrosSubscriber)):
                if (sub.enable_ros_subscribe):
                    raw_msg_type = PyNitrosSubscriber.BRIDGE_TYPE_TO_RAW_TYPE[sub.message_type]
                    rclpy_subs.append(Subscriber(self.node, raw_msg_type, sub.sub_topic_name))
                else:
                    rclpy_subs.append(Subscriber(self.node, sub.message_type, sub.sub_topic_name))
            else:
                rclpy_subs.append(sub)

        self._cuda_memblock_fd_to_ptr = {}
        self._event_handle_to_event = {}
        self._cuda_memblock_handle = []
        self.input_callback = callback

        if (synchronizer is TimeSynchronizer):
            rclpy_synchronizer = TimeSynchronizer(rclpy_subs, queue_size)
        elif (synchronizer is ApproximateTimeSynchronizer):
            rclpy_synchronizer = ApproximateTimeSynchronizer(rclpy_subs, queue_size, slop)
        else:
            raise ValueError('Unsupported synchronizer type')
        rclpy_synchronizer.registerCallback(self.filter_callback)

    def filter_callback(self, *args):
        msg_views = []
        for index in range(len(args)):
            cur_message = args[index]
            cur_subscriber = self.subscribers[index]

            if (isinstance(cur_subscriber, PyNitrosSubscriber)):
                # Create PyNITROS view from bridge msg
                bridge_message_type = cur_subscriber.message_type
                if (cur_subscriber.enable_ros_subscribe):
                    pynitros_view = PyNitrosSubscriber.BRIDGE_TYPE_TO_PYNITROS_VIEW[
                                                        bridge_message_type](cur_message)
                else:
                    if cur_message.cuda_event_handle:
                        event_handle_tuple = tuple(cur_message.cuda_event_handle)
                        if event_handle_tuple not in self._event_handle_to_event:
                            # Save event handle to local map
                            event_handle = cudart.cudaIpcEventHandle_t()
                            event_handle.reserved = cur_message.cuda_event_handle

                            err, cuda_event = cudart.cudaIpcOpenEventHandle(event_handle)
                            self._event_handle_to_event[event_handle_tuple] = cuda_event
                            self.ASSERT_CUDA_SUCCESS(err)
                        else:
                            cuda_event = self._event_handle_to_event[event_handle_tuple]
                        # Synchronize to upstream Event
                        err, = cudart.cudaEventSynchronize(cuda_event)
                        self.ASSERT_CUDA_SUCCESS(err)

                    # Create PyNITROS view
                    sender_pid, memblock_fd = \
                        PyNitrosSubscriber.BRIDGE_TYPE_TO_PYNITROS_VIEW[
                            bridge_message_type].get_pid_fd(cur_message)
                    if (sender_pid, memblock_fd) not in self._cuda_memblock_fd_to_ptr:
                        try:
                            pynitros_view = PyNitrosSubscriber.BRIDGE_TYPE_TO_PYNITROS_VIEW[
                                                                bridge_message_type](cur_message)
                        except ValueError:
                            self.node.get_logger().warn(
                                'Failed to create PyNITROS view from bridge msg')
                            return
                        gpu_ptr = pynitros_view.get_buffer()
                        gpu_handle = pynitros_view.get_handle()
                        self._cuda_memblock_fd_to_ptr[(sender_pid, memblock_fd)] = gpu_ptr
                        self._cuda_memblock_handle.append(gpu_handle)
                    else:
                        # Found gpu_ptr from map and create PyNITROS view
                        gpu_ptr = self._cuda_memblock_fd_to_ptr[(sender_pid, memblock_fd)]
                        try:
                            pynitros_view = PyNitrosSubscriber.BRIDGE_TYPE_TO_PYNITROS_VIEW[
                                            bridge_message_type](cur_message, gpu_ptr)
                        except ValueError:
                            self.node.get_logger().warn(
                                'Failed to create PyNITROS view from bridge msg')
                            return
                msg_views.append(pynitros_view)
            else:
                # Passthrough raw message
                msg_views.append(cur_message)

        self.input_callback(*msg_views)
        for pynitros_view in msg_views:
            if isinstance(pynitros_view, PyNitrosTypeViewBase):
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
