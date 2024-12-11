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

import os
import uuid

from cuda import cuda, cudart
from isaac_ros_nitros_bridge_interfaces.msg import NitrosBridgeImage
import numpy as np
from sensor_msgs.msg import Image

from .pynitros_type_builder_base import PyNitrosTypeBuilder


class PyNitrosImageBuilder(PyNitrosTypeBuilder):
    """Class to build NitrosBridgeImage messages."""

    def __init__(self, num_buffer=10, timeout=1):
        """
        Initialize the PyNitrosTensorListBuilder.

        Parameters
        ----------
        num_buffer: int
            Number of buffers to create
        timeout: int
            Timeout in ms for recycling buffers with refcount 0

        """
        super().__init__(num_buffer, timeout)

    def build(self, image_data, image_height, image_width, image_step,
              image_encoding, image_header,
              device_id, enable_ros_publish, event=None):
        """
        Build NITROS Image message.

        Parameters
        ----------
        image_data: cuda.CUdeviceptr
            Device pointer to image data
        image_height: int
            Height of the image
        image_width: int
            Width of the image
        image_step: int
            Step of the image in bytes
        image_encoding: str
            Encoding of the image
        image_header: Header
            Header for the image
        device_id: int
            Device id the tensors are residing on
        enable_ros_publish: bool
            Enable ROS publishing
        event: cudaEvent
            Event with unfinished CUDA operations to synchronize on

        Returns
        -------
        tuple: (NitrosBridgeImage, Image)
            NitrosBridgeImage: NitrosBridgeImage message
            Image: Image message, None if enable_ros_publish is False

        """
        if self._first_frame:
            buffer_size = image_step * image_height
            super().create_memblock_pool(buffer_size)
            self._first_frame = False

        # Synchronize to user Event
        if event:
            err, = cudart.cudaEventSynchronize(self.event)
            super().ASSERT_CUDA_SUCCESS(err)

        # Create Nitros Bridge Image
        nitros_bridge_image_msg = NitrosBridgeImage()
        nitros_bridge_image_msg.header = image_header
        nitros_bridge_image_msg.height = image_height
        nitros_bridge_image_msg.width = image_width
        nitros_bridge_image_msg.encoding = image_encoding
        nitros_bridge_image_msg.is_bigendian = 0
        nitros_bridge_image_msg.step = image_step

        memblock_fd, new_memblock_uuid, image_event_handle = self._setup_image_memblock(
            image_data)
        nitros_bridge_image_msg.data = [os.getpid(), memblock_fd]
        nitros_bridge_image_msg.cuda_event_handle = image_event_handle.reserved
        nitros_bridge_image_msg.device_id = device_id
        nitros_bridge_image_msg.uid = str(new_memblock_uuid)

        # Setup Shareable Memory To Transport Image
        image_msg_raw = None

        if enable_ros_publish:
            image_msg_raw = Image()
            image_size = image_step * image_height
            # Set image properties
            image_msg_raw.header = image_header
            image_msg_raw.height = image_height
            image_msg_raw.width = image_width
            image_msg_raw.encoding = image_encoding
            image_msg_raw.is_bigendian = 0
            image_msg_raw.step = image_step
            # Assign to the private data field to reduce overhead of msg data assignment
            image_msg_raw._data = np.zeros((image_height, image_width, 3), dtype=np.uint8)
            err, = cudart.cudaMemcpy(image_msg_raw.data, image_data, image_size,
                                     cudart.cudaMemcpyKind.cudaMemcpyDeviceToHost)
            super().ASSERT_CUDA_SUCCESS(err)

        # Return Nitros Bridge Image
        return (nitros_bridge_image_msg, image_msg_raw)

    def _setup_image_memblock(self, image_tensor_ptr):
        """Set up shareable CUDA memblocks for images."""
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

        err, = cuda.cuMemcpyDtoDAsync(cuda.CUdeviceptr(
            cuda_memblock_virtual_ptr), image_tensor_ptr, self.buffer_size, self._stream)
        super().ASSERT_CUDA_SUCCESS(err)

        err, = cudart.cudaEventRecord(cuda_event, self._stream)
        super().ASSERT_CUDA_SUCCESS(err)

        return (cuda_memblock_fd, new_cuda_memblock_uuid, cuda_event_handle)
