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

from cuda import cudart
import torch

from .pynitros_type_view_base import PyNitrosTypeViewBase


class PyNitrosImageView(PyNitrosTypeViewBase):
    """PyNITROS View for NITROSBridgeImage."""

    def __init__(self, raw_msg, gpu_ptr=None):
        super().__init__(raw_msg, gpu_ptr)

        if (isinstance(self.raw_msg, self.nitros_bridge_msg_types)):
            sender_pid, memblock_fd = self.get_pid_fd(self.raw_msg)
            if (self._open_shm_and_check_uid(sender_pid, memblock_fd) is not True):
                raise ValueError('UID match failed')

        if (self.gpu_ptr is None):
            if (isinstance(self.raw_msg, self.nitros_bridge_msg_types)):
                self.gpu_ptr = self._from_bridge_msg()
            elif (isinstance(self.raw_msg, self.raw_msg_types)):
                self.gpu_ptr = self._from_raw_msg()
            else:
                raise RuntimeError('Invalid message type')

        self.gpu_ptr = int(self.gpu_ptr)
        self.shape = torch.Size(
            [raw_msg.height * raw_msg.step])
        self.__cuda_array_interface__ = {
            'shape': self.shape,
            'typestr': '|u1',
            'data': (self.gpu_ptr, False),
            'version': 0
        }

    @staticmethod
    def get_pid_fd(bridge_msg):
        return bridge_msg.data[0], bridge_msg.data[1]

    def _from_bridge_msg(self):
        sender_pid, memblock_fd = self.get_pid_fd(self.raw_msg)
        data_size = self.raw_msg.height * self.raw_msg.step
        return self._import_gpu_ptr_from_fd(sender_pid, memblock_fd, data_size)

    def _from_raw_msg(self):
        image_size = len(self.raw_msg.data)
        err, device_ptr = cudart.cudaMalloc(image_size)
        self.ASSERT_CUDA_SUCCESS(err)

        err, = cudart.cudaMemcpy(device_ptr, self.raw_msg.data, image_size,
                                 cudart.cudaMemcpyKind.cudaMemcpyHostToDevice)
        self.ASSERT_CUDA_SUCCESS(err)
        return device_ptr

    def get_size_in_bytes(self):
        return self.raw_msg.step * self.raw_msg.height

    def get_width(self):
        return self.raw_msg.width

    def get_height(self):
        return self.raw_msg.height

    def get_stride(self):
        return self.raw_msg.step

    def get_buffer(self):
        return self.gpu_ptr

    def get_encoding(self):
        return self.raw_msg.encoding
