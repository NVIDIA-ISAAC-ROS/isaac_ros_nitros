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
import time
import uuid

from cuda import cuda, cudart
from isaac_ros_pynitros.utils.cpu_shared_mem import CPUSharedMem
import rclpy


class PyNitrosTypeBuilder():
    """
    Base class of PyNITROS message builders.

    The builder creates a pool of sharable GPU memory buffers during initialization.
    to hold the GPU data of the message. The inherited class should implement
    the build method to build different type of NITROS Bridge messages.
    """

    def __init__(self, num_buffer=10, timeout=1):
        """
        Initialize the PyNitrosTypeBuilder.

        Parameters
        ----------
        num_buffer: int
            Number of buffers to create
        timeout: int
            Timeout in ms for recycling buffers with refcount 0

        """
        self.num_buffer = num_buffer
        self.timeout = timeout

        err, = cuda.cuInit(0)
        self.ASSERT_CUDA_SUCCESS(err)

        # Create CUDA Stream
        err, self._stream = cudart.cudaStreamCreate()
        self.ASSERT_CUDA_SUCCESS(err)

        # Maps CUDA Memblocks to their respective shared CPU memory
        self._cuda_memblock_to_cpu_mem = {}
        self._cuda_memblock_event = {}

        # Cuda MemBlock Housekeeping
        self._cuda_memblock_fd_to_ptr = {}
        self._cuda_memblock_info = []
        self._cuda_memblock_start_time = {}
        self._mem_block_index = 0

        self._first_frame = True

    def create_memblock_pool(self, buffer_size):
        """
        Create a pool of CUDA IPC memory blocks.

        This pool is created the first time message is built.
        """
        self.buffer_size = buffer_size
        for _ in range(self.num_buffer):
            # Create memory block
            cuda_memblock_fd, cuda_memblock_virtual_ptr = self._create_cuda_memblock()

            # Add to mem pool mapping
            cpu_memblock_info = CPUSharedMem()
            cpu_memblock_info.create_shm(str(os.getpid()) + str(cuda_memblock_fd))

            cpu_memblock_info.lock.acquire()
            cpu_memblock_info.put_value((0, uuid.uuid4()))
            cpu_memblock_info.lock.release()

            self._cuda_memblock_to_cpu_mem[cuda_memblock_fd] = cpu_memblock_info

            # Add fd, ptr info
            self._cuda_memblock_fd_to_ptr[cuda_memblock_fd] = int(
                cuda_memblock_virtual_ptr)

            # Set timeout value
            self._cuda_memblock_start_time[cuda_memblock_fd] = 0

            # Setup event, record stream, and perform copy
            err, event = cudart.cudaEventCreateWithFlags(
                cudart.cudaEventInterprocess | cudart.cudaEventDisableTiming)
            self.ASSERT_CUDA_SUCCESS(err)

            err, event_handle = cudart.cudaIpcGetEventHandle(event)
            self.ASSERT_CUDA_SUCCESS(err)
            self._cuda_memblock_event[cuda_memblock_fd] = (event, event_handle)

    def get_free_memblock(self):
        """
        Get a free CUDA memblock from the pool.

        If no free memblock is available, it will keep looping until a
        free memblock is available.

        Returns
        -------
        Tuple of (memblock_fd, memblock_uuid, memblock_refcount)
            memblock_fd: int
                File descriptor of the memblock
            memblock_uuid: uuid
                UID of the memblock, updated everytime the memblock is reused
            memblock_refcount: int
                Reference count of the memblock

        """
        cuda_memblock_list = list(self._cuda_memblock_to_cpu_mem.items())
        last_used_index = self._mem_block_index
        while True:
            cuda_memblock_fd, cuda_memblock = cuda_memblock_list[self._mem_block_index]
            start_time = self._cuda_memblock_start_time[cuda_memblock_fd]
            current_time = time.time() * 1000

            # TODO(yuankunz): Reset timeout everytime refcount is decreased to 0
            if current_time - start_time > self.timeout:
                cuda_memblock.lock.acquire()
                cuda_memblock_refcount, cuda_memblock_uuid = cuda_memblock.get_value()
                if cuda_memblock_refcount == 0:
                    self._cuda_memblock_start_time[cuda_memblock_fd] = time.time() * 1000
                    cuda_memblock.lock.release()
                    self._mem_block_index = (self._mem_block_index + 1) % len(cuda_memblock_list)
                    return cuda_memblock_fd, cuda_memblock_uuid, cuda_memblock_refcount
                else:
                    cuda_memblock.lock.release()
            self._mem_block_index = (self._mem_block_index + 1) % len(cuda_memblock_list)
            if self._mem_block_index == last_used_index:
                rclpy.logging.get_logger('PyNITROSBuilder').info(
                    'No free memblock available, retry after 1ms...')
                time.sleep(0.001)

    def _create_cuda_memblock(self):
        """Create an IPC CUDA memory pool with the given buffer size and number of buffers."""
        # Allocate GPU Space -> CUmemGenericAllocationHandle
        allocProp = cuda.CUmemAllocationProp()
        allocProp.type = cuda.CUmemAllocationType.CU_MEM_ALLOCATION_TYPE_PINNED
        allocProp.location.type = cuda.CUmemLocationType.CU_MEM_LOCATION_TYPE_DEVICE
        allocProp.requestedHandleTypes = \
            cuda.CUmemAllocationHandleType.CU_MEM_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR
        allocProp.location.id = 0

        # Get granularity
        err, granularity = cuda.cuMemGetAllocationGranularity(
            allocProp, cuda.CUmemAllocationGranularity_flags.CU_MEM_ALLOC_GRANULARITY_MINIMUM)
        self.ASSERT_CUDA_SUCCESS(err)

        # Call cuMemCreate with the address of allocProp
        rounded_data_size = self._round_up(self.buffer_size, granularity)

        err, handle = cuda.cuMemCreate(rounded_data_size, allocProp, 0)
        self.ASSERT_CUDA_SUCCESS(err)

        # Reserve Virtual Address Range
        err, memblock_virtual_ptr = cuda.cuMemAddressReserve(
            rounded_data_size, 0, 0, 0)
        self.ASSERT_CUDA_SUCCESS(err)

        # Map Handle to Virtual Address Range, Set RW Access
        err, = cuda.cuMemMap(memblock_virtual_ptr,
                             rounded_data_size, 0, handle, 0)
        self.ASSERT_CUDA_SUCCESS(err)

        self._cuda_memblock_info.append(
            (handle, memblock_virtual_ptr, rounded_data_size))

        accessDesc = cuda.CUmemAccessDesc()
        accessDesc.location = allocProp.location
        accessDesc.flags = cuda.CUmemAccess_flags.CU_MEM_ACCESS_FLAGS_PROT_READWRITE

        err, = cuda.cuMemSetAccess(
            memblock_virtual_ptr, rounded_data_size, [accessDesc], 1)
        self.ASSERT_CUDA_SUCCESS(err)

        # Export Shareable Handle
        err, memblock_fd = cuda.cuMemExportToShareableHandle(
            handle, cuda.CUmemAllocationHandleType.CU_MEM_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR, 0)
        self.ASSERT_CUDA_SUCCESS(err)

        return (memblock_fd, memblock_virtual_ptr)

    def _round_up(self, size, granularity):
        """Round up the size to the nearest multiple of granularity."""
        mult = math.ceil(size / float(granularity))
        return mult * granularity

    def ASSERT_CUDA_SUCCESS(self, err):
        """Assert if the CUDA operation is successful."""
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
        """Destructor to free all the allocated resources."""
        for cpu_memblock in self._cuda_memblock_to_cpu_mem.values():
            cpu_memblock.close()

        for cuda_event, _ in self._cuda_memblock_event.values():
            err, = cudart.cudaEventDestroy(cuda_event)
            self.ASSERT_CUDA_SUCCESS(err)

        err, = cuda.cuStreamDestroy(self._stream)
        self.ASSERT_CUDA_SUCCESS(err)

        # Free shared cuda memblocks
        for cuda_memblock_handle, cuda_memblock_virtual_ptr, data_size in self._cuda_memblock_info:
            err, = cuda.cuMemUnmap(cuda_memblock_virtual_ptr, data_size)
            self.ASSERT_CUDA_SUCCESS(err)

            err, = cuda.cuMemAddressFree(cuda_memblock_virtual_ptr, data_size)
            self.ASSERT_CUDA_SUCCESS(err)

            err, = cuda.cuMemRelease(cuda_memblock_handle)
            self.ASSERT_CUDA_SUCCESS(err)
