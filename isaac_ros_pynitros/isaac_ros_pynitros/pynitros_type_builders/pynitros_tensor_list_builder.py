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
import os
import uuid

from cuda import cuda, cudart

from isaac_ros_nitros_bridge_interfaces.msg import NitrosBridgeTensorList
from isaac_ros_pynitros.utils.tensor_data_type import TensorDataTypeUtils
from isaac_ros_tensor_list_interfaces.msg import Tensor, TensorList, TensorShape
import numpy as np

from .pynitros_type_builder_base import PyNitrosTypeBuilder


class PyNitrosTensorListBuilder(PyNitrosTypeBuilder):
    """Class to build NitrosBridgeTensorList messages."""

    def __init__(self, num_buffer=10, timeout=1):
        """
        Initialize the PyNitrosTensorListBuilder.

        Parameters
        ----------
        buffer_size: int
            Size of each buffer in bytes
        num_buffer: int
            Number of buffers to create
        timeout: int
            Timeout in ms for recycling buffers with refcount 0

        """
        super().__init__(num_buffer, timeout)

    def build(self, built_tensors, tensors_header,
              device_id, enable_ros_publish, event=None):
        """
        Build NitrosBridgeTensorList message.

        Parameters
        ----------
        built_tensors: list
            List of built tensors
        tensors_header: Header
            Header for the tensor list
        device_id: int
            device id the tensors are residing on
        enable_ros_publish: bool
            enable ROS publishing
        event: cudaEvent
            event with unfinished CUDA operations to synchronize on

        Returns
        -------
        tuple: (NitrosBridgeTensorList, TensorList)
            NitrosBridgeTensorList: NitrosBridgeTensorList message
            TensorList: TensorList message, None if enable_ros_publish is False

        """
        nitros_bridge_tensor_list_msg = NitrosBridgeTensorList()
        nitros_bridge_tensor_list_msg.header = tensors_header
        nitros_bridge_tensor_list_msg.tensors = []

        tensor_ptrs_sizes = []
        tensor_list_msg_raw = None
        if enable_ros_publish:
            tensor_list_msg_raw = TensorList()
            tensor_list_msg_raw.header = tensors_header
            tensor_list_msg_raw.tensors = []

        if self._first_frame:
            total_tensor_size = 0
            for tensor in built_tensors:
                total_tensor_size += tensor.strides[0] * tensor.shape.dims[0]
            super().create_memblock_pool(total_tensor_size)
            self._first_frame = False

        # Synchronize to upstream Event
        if event:
            err, = cudart.cudaEventSynchronize(self.event)
            super().ASSERT_CUDA_SUCCESS(err)

        # Set the data field
        index = 0
        for tensor in built_tensors:
            nitros_bridge_tensor_list_msg.tensors.append(tensor)
            tensor_size = tensor.strides[0] * tensor.shape.dims[0]

            tensor_gpu_ptr = int.from_bytes(tensor.data, byteorder='big')
            tensor_ptrs_sizes.append((tensor_gpu_ptr, tensor_size))
            if enable_ros_publish:
                raw_tensor = Tensor()
                raw_tensor.name = tensor.name
                raw_tensor.shape = tensor.shape
                raw_tensor.data_type = tensor.data_type
                raw_tensor.strides = tensor.strides
                # Assign to the private data field to reduce overhead of msg data assignment
                raw_tensor._data = np.empty(tensor_size, dtype=np.uint8).tobytes()
                err, = cudart.cudaMemcpy(raw_tensor.data, tensor_gpu_ptr, tensor_size,
                                         cudart.cudaMemcpyKind.cudaMemcpyDeviceToHost)
                super().ASSERT_CUDA_SUCCESS(err)
                tensor_list_msg_raw.tensors.append(raw_tensor)
            index += 1

        memblock_fd, new_memblock_uuid, tensor_event_handle = self._setup_tensors_memblock(
            tensor_ptrs_sizes)
        nitros_bridge_tensor_list_msg.pid = os.getpid()
        nitros_bridge_tensor_list_msg.fd = memblock_fd
        nitros_bridge_tensor_list_msg.cuda_event_handle = tensor_event_handle.reserved
        nitros_bridge_tensor_list_msg.device_id = device_id
        nitros_bridge_tensor_list_msg.uid = str(new_memblock_uuid)

        return (nitros_bridge_tensor_list_msg, tensor_list_msg_raw)

    def build_tensor(self, tensor_data, tensor_name, tensor_shape, tensor_data_type):
        """
        Build a NitrosBridgeTensor message.

        Parameters
        ----------
        tensor_data: device ptr
            Device pointer to the tensor data
        tensor_name: str
            Name of the tensor
        tensor_shape: list
            Shape of the tensor
        tensor_data_type: int
            Data type of the tensor

        Returns
        -------
        tensor: NitrosBridgeTensor message

        """
        # Build Tensor Messages
        tensor_shape_msg = TensorShape()
        tensor_shape_msg.rank = len(tensor_shape)
        tensor_shape_msg.dims = tensor_shape

        nitros_bridge_tensor_msg = Tensor()
        nitros_bridge_tensor_msg.name = tensor_name
        nitros_bridge_tensor_msg.shape = tensor_shape_msg
        nitros_bridge_tensor_msg.data_type = tensor_data_type

        strides = []
        element_size = TensorDataTypeUtils.get_size_in_bytes(tensor_data_type)
        for i in range(len(tensor_shape)):
            if i == (len(tensor_shape)-1):
                strides.append(element_size)
            else:
                strides.append(math.prod(tensor_shape[i+1:])*element_size)

        nitros_bridge_tensor_msg.strides = strides
        nitros_bridge_tensor_msg.data = tensor_data.to_bytes(8, byteorder='big')

        return nitros_bridge_tensor_msg

    def _setup_tensors_memblock(self, tensor_ptrs_sizes):
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

        # Copy tensors back-to-back into cuda mem space
        offset = 0
        for cur_tensor_ptr, cur_tensor_size in tensor_ptrs_sizes:
            cur_gpu_ptr = cuda.CUdeviceptr(cuda_memblock_virtual_ptr + offset)
            err, = cuda.cuMemcpyDtoDAsync(
                cur_gpu_ptr, cur_tensor_ptr, cur_tensor_size, self._stream)
            super().ASSERT_CUDA_SUCCESS(err)

            offset += cur_tensor_size

        err, = cudart.cudaEventRecord(cuda_event, self._stream)
        super().ASSERT_CUDA_SUCCESS(err)

        return (cuda_memblock_fd, new_cuda_memblock_uuid, cuda_event_handle)
