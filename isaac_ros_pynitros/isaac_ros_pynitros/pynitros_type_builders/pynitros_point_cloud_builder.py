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

import os
import uuid

import cuda.bindings.driver as cuda
import cuda.bindings.runtime as cudart
from isaac_ros_nitros_bridge_interfaces.msg import NitrosBridgePointCloud
import numpy as np
from sensor_msgs.msg import PointCloud2

from .pynitros_type_builder_base import PyNitrosTypeBuilder


class PyNitrosPointCloudBuilder(PyNitrosTypeBuilder):
    """Class to build NitrosBridgePointCloud messages."""

    def __init__(self, num_buffer=10, timeout=1):
        """
        Initialize the PyNitrosPointCloudBuilder.

        Parameters
        ----------
        num_buffer: int
            Number of buffers to create
        timeout: int
            Timeout in ms for recycling buffers with refcount 0

        """
        super().__init__(num_buffer, timeout)

    def build(self, points_data, height, width, fields, is_bigendian, point_step,
              row_step, is_dense, header, device_id, enable_ros_publish, event=None):
        """
        Build NitrosBridgePointCloud message.

        Parameters
        ----------
        points_data: cuda.CUdeviceptr
            Device pointer to point cloud data
        height: int
            Height of the point cloud
        width: int
            Width of the point cloud
        fields: list
            List of PointField objects
        is_bigendian: bool
            Whether the point cloud is big endian
        point_step: int
            Step size in bytes for each point
        row_step: int
            Step size in bytes for each row
        is_dense: bool
            Whether the point cloud is dense
        header: Header
            Header for the point cloud
        device_id: int
            device id the point cloud is residing on
        enable_ros_publish: bool
            enable ROS publishing
        event: cudaEvent
            event with unfinished CUDA operations to synchronize on

        Returns
        -------
        tuple: (NitrosBridgePointCloud, PointCloud2)
            NitrosBridgePointCloud: NitrosBridgePointCloud message
            PointCloud2: PointCloud2 message, None if enable_ros_publish is False

        """
        # Synchronize to user Event
        if event:
            err, = cudart.cudaEventSynchronize(event)
            super().ASSERT_CUDA_SUCCESS(err)

        nitros_bridge_point_cloud_msg = NitrosBridgePointCloud()
        nitros_bridge_point_cloud_msg.header = header
        nitros_bridge_point_cloud_msg.height = height
        nitros_bridge_point_cloud_msg.width = width
        nitros_bridge_point_cloud_msg.fields = fields
        nitros_bridge_point_cloud_msg.is_bigendian = is_bigendian
        nitros_bridge_point_cloud_msg.point_step = point_step
        nitros_bridge_point_cloud_msg.row_step = row_step
        nitros_bridge_point_cloud_msg.is_dense = is_dense

        point_cloud_msg_raw = None
        if enable_ros_publish:

            point_cloud_msg_raw = PointCloud2()
            point_cloud_msg_raw.header = header
            point_cloud_msg_raw.height = height
            point_cloud_msg_raw.width = width
            point_cloud_msg_raw.is_bigendian = is_bigendian
            point_cloud_msg_raw.is_dense = is_dense
            point_cloud_msg_raw.fields = fields
            point_cloud_msg_raw.point_step = point_step
            point_cloud_msg_raw.row_step = row_step

        # Calculate required buffer size for this frame using ROS2 standard
        required_buffer_size = row_step * height  # Use ROS2 standard calculation

        if self._first_frame:
            super().create_memblock_pool(required_buffer_size)
            self._first_frame = False

        # Set the Nitros-specific fields
        # Note: points_data, num_points, use_color are not part of the message definition
        # These are handled internally by the builder for GPU memory management

        if enable_ros_publish and point_cloud_msg_raw is not None:
            # Copy data from device to host for ROS message
            data_size = row_step * height  # Use ROS2 standard calculation
            host_buf = np.empty(data_size, dtype=np.uint8)
            err, = cudart.cudaMemcpy(
                host_buf.ctypes.data,
                points_data,
                data_size,
                cudart.cudaMemcpyKind.cudaMemcpyDeviceToHost)
            super().ASSERT_CUDA_SUCCESS(err)
            point_cloud_msg_raw.data = host_buf.tobytes()

        memblock_fd, new_memblock_uuid, point_cloud_event_handle = (
            self._setup_point_cloud_memblock(points_data, row_step * height))

        # Set Nitros-specific fields for GPU memory management
        nitros_bridge_point_cloud_msg.data = [os.getpid(), memblock_fd]  # [PID, FD]
        nitros_bridge_point_cloud_msg.cuda_event_handle = point_cloud_event_handle.reserved
        nitros_bridge_point_cloud_msg.device_id = device_id
        nitros_bridge_point_cloud_msg.uid = str(new_memblock_uuid)

        return (nitros_bridge_point_cloud_msg, point_cloud_msg_raw)

    def _setup_point_cloud_memblock(self, gpu_ptr, data_size):
        """Set up sharable device memory blocks."""
        cuda_memblock_fd, cuda_memblock_uuid, cuda_memblock_refcount = super().get_free_memblock()
        cuda_memblock_virtual_ptr = self._cuda_memblock_fd_to_ptr[cuda_memblock_fd]
        cuda_event, cuda_event_handle = self._cuda_memblock_event[cuda_memblock_fd]

        # Update memblock uuid header
        new_cuda_memblock_uuid = uuid.uuid4()
        cuda_memblock_cpu_mem = self._cuda_memblock_to_cpu_mem[cuda_memblock_fd]

        cuda_memblock_cpu_mem.lock.acquire()
        cuda_memblock_cpu_mem.put_value(
            (cuda_memblock_refcount, new_cuda_memblock_uuid))
        cuda_memblock_cpu_mem.lock.release()

        # Copy point cloud data into cuda mem space
        # data_size is now passed as a parameter (calculated as row_step * height)

        # Validate that data size doesn't exceed buffer capacity
        if data_size > self.buffer_size:
            raise ValueError(
                f'Point cloud data size ({data_size} bytes) exceeds buffer capacity '
                f'({self.buffer_size} bytes). This should not happen as the pool should '
                'have been resized.')

        # Copy data to the memblock
        err, = cuda.cuMemcpyDtoDAsync(
            cuda_memblock_virtual_ptr, gpu_ptr, data_size, self._stream)
        super().ASSERT_CUDA_SUCCESS(err)

        # Record event after the copy
        err, = cudart.cudaEventRecord(cuda_event, self._stream)
        super().ASSERT_CUDA_SUCCESS(err)

        # Synchronize stream to ensure the event is recorded
        err, = cudart.cudaStreamSynchronize(self._stream)
        super().ASSERT_CUDA_SUCCESS(err)

        return (cuda_memblock_fd, new_cuda_memblock_uuid, cuda_event_handle)
