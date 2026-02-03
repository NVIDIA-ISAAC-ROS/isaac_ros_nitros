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

import cuda.bindings.runtime as cudart

from .pynitros_type_view_base import PyNitrosTypeViewBase


class PyNitrosPointCloudView(PyNitrosTypeViewBase):
    """PyNITROS view for NitrosBridgePointCloud."""

    def __init__(self, raw_msg, gpu_ptr=None):
        super().__init__(raw_msg, gpu_ptr)

        if isinstance(self.raw_msg, self.nitros_bridge_msg_types):
            sender_pid, memblock_fd = self.get_pid_fd(self.raw_msg)
            if (self._open_shm_and_check_uid(sender_pid, memblock_fd)
                    is not True):
                raise ValueError('UID match failed')

        # Initialize GPU pointer first
        if self.gpu_ptr is None:
            if isinstance(self.raw_msg, self.nitros_bridge_msg_types):
                self.gpu_ptr = self._from_bridge_msg()
            elif isinstance(self.raw_msg, self.raw_msg_types):
                self.gpu_ptr = self._from_raw_msg()
            else:
                raise RuntimeError('Invalid message type')
        self.gpu_ptr = int(self.gpu_ptr)

    @staticmethod
    def get_pid_fd(bridge_msg):
        return bridge_msg.data[0], bridge_msg.data[1]

    def _from_bridge_msg(self):
        sender_pid, memblock_fd = self.get_pid_fd(self.raw_msg)
        data_size = self.raw_msg.height * self.raw_msg.row_step
        return self._import_gpu_ptr_from_fd(sender_pid, memblock_fd, data_size)

    def _from_raw_msg(self):
        data_size = len(self.raw_msg.data)
        err, device_ptr = cudart.cudaMalloc(data_size)
        self.ASSERT_CUDA_SUCCESS(err)
        err, = cudart.cudaMemcpy(device_ptr, self.raw_msg.data, data_size,
                                 cudart.cudaMemcpyKind.cudaMemcpyHostToDevice)
        self.ASSERT_CUDA_SUCCESS(err)
        return device_ptr

    def get_height(self):
        return self.raw_msg.height

    def get_width(self):
        return self.raw_msg.width

    def get_fields(self):
        return self.raw_msg.fields

    def is_bigendian(self):
        return self.raw_msg.is_bigendian

    def get_point_step(self):
        return self.raw_msg.point_step

    def get_row_step(self):
        return self.raw_msg.row_step

    def is_dense(self):
        return self.raw_msg.is_dense

    def get_point_count(self):
        """Get the number of points in the point cloud."""
        return self.raw_msg.height * self.raw_msg.width

    def has_color(self):
        """Check if the point cloud has color information."""
        # Check if there are color-related fields in the point cloud
        if hasattr(self.raw_msg, 'fields') and self.raw_msg.fields:
            for field in self.raw_msg.fields:
                if field.name.lower() in ['rgb', 'r', 'g', 'b', 'rgba']:
                    return True

        # Check if point_step indicates color data (16 bytes for XYZ + RGB vs 12 bytes
        # for XYZ only)
        if hasattr(self.raw_msg, 'point_step'):
            return self.raw_msg.point_step > 12

        return False

    def get_points_data(self):
        """Get the GPU pointer to the points data."""
        return self.gpu_ptr

    def get_device_id(self):
        """Get the device ID where the point cloud data resides."""
        return getattr(self.raw_msg, 'device_id', 0)

    def get_buffer(self):
        return self.gpu_ptr

    def get_event(self):
        """Get the CUDA event associated with this point cloud."""
        if hasattr(self, 'event'):
            return self.event
        return None
