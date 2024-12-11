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

import math

from cuda import cuda, cudart
from isaac_ros_pynitros.utils.tensor_data_type import TensorDataTypeUtils
import torch

from .pynitros_type_view_base import PyNitrosTypeViewBase


class PyNitrosTensorListView(PyNitrosTypeViewBase):
    """PyNITROS view for NitrosBridgeTensorList."""

    def __init__(self, raw_msg, gpu_ptr=None):
        super().__init__(raw_msg, gpu_ptr)
        self._tensor_map = {}
        self._tensors = []
        self._total_tensor_size = 0

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
        self._prepare_tensor_view()

    @staticmethod
    def get_pid_fd(bridge_msg):
        return bridge_msg.pid, bridge_msg.fd

    def _prepare_tensor_view(self):
        if (not self.gpu_ptr):
            raise RuntimeError('Invalid GPU pointer')
        for nitros_bridge_tensor_msg in self.raw_msg.tensors:
            cur_gpu_ptr = cuda.CUdeviceptr(int(self.gpu_ptr) + self._total_tensor_size)
            pynitros_tensor_view = PyNitrosTensorView(
                nitros_bridge_tensor_msg, cur_gpu_ptr)
            tensor_size = pynitros_tensor_view.get_tensor_size()
            self._total_tensor_size += tensor_size

            self._tensor_map[nitros_bridge_tensor_msg.name] = pynitros_tensor_view
            self._tensors.append(pynitros_tensor_view)

    def _from_bridge_msg(self):
        sending_pid = self.raw_msg.pid
        memblock_fd = self.raw_msg.fd
        data_size = 0
        for tensor in self.raw_msg.tensors:
            data_size += \
                math.prod(tensor.shape.dims) * \
                TensorDataTypeUtils.get_size_in_bytes(tensor.data_type)
        virtual_ptr = self._import_gpu_ptr_from_fd(sending_pid, memblock_fd, data_size)
        return virtual_ptr

    def _from_raw_msg(self):
        total_tensor_size, offset = 0, 0
        for tensor in self.raw_msg.tensors:
            total_tensor_size += \
                math.prod(tensor.shape.dims) * \
                TensorDataTypeUtils.get_size_in_bytes(tensor.data_type)
        err, device_ptr = cudart.cudaMalloc(total_tensor_size)
        self.ASSERT_CUDA_SUCCESS(err)

        for tensor in self.raw_msg.tensors:
            tensor_size = \
                math.prod(tensor.shape.dims) * \
                TensorDataTypeUtils.get_size_in_bytes(tensor.data_type)
            err, = cudart.cudaMemcpy(device_ptr+offset, tensor.data, tensor_size,
                                     cudart.cudaMemcpyKind.cudaMemcpyHostToDevice)
            self.ASSERT_CUDA_SUCCESS(err)
            offset += tensor_size
        return device_ptr

    def get_buffer(self):
        return self.gpu_ptr

    def get_tensor_count(self):
        return len(self.tensors)

    def get_named_tensor(self, name):
        return self._tensor_map[name]

    def get_all_tensors(self):
        return self._tensors

    def get_size_in_bytes(self):
        return self._total_tensor_size


class PyNitrosTensorView(PyNitrosTypeViewBase):
    """PyNITROS view for NitrosBridgeTensor."""

    def __init__(self, raw_msg, gpu_ptr):
        super().__init__(raw_msg, gpu_ptr)
        self.shape = torch.Size(self.raw_msg.shape.dims)
        self.gpu_ptr = int(self.gpu_ptr)
        self.element_count = math.prod(self.raw_msg.shape.dims)
        self.__cuda_array_interface__ = {
            'shape': self.shape,
            'strides': self.raw_msg.strides,
            'typestr': TensorDataTypeUtils.get_typestr(self.raw_msg.data_type),
            'data': (self.gpu_ptr, False),
            'version': 0
        }

    def get_name(self):
        return self.raw_msg.name

    def get_buffer(self):
        return self.gpu_ptr

    def get_rank(self):
        return self.raw_msg.shape.rank

    def get_bytes_per_element(self):
        return TensorDataTypeUtils.get_size_in_bytes(self.raw_msg.data_type)

    def get_element_count(self):
        return self.element_count

    def get_tensor_size(self):
        return self.get_element_count() * self.get_bytes_per_element()

    def get_shape(self):
        return tuple(self.raw_msg.shape.dims)

    def get_element_type(self):
        return self.raw_msg.data_type
