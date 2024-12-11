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

import ctypes
from ctypes import c_long
import math

from cuda import cuda, cudart
from isaac_ros_nitros_bridge_interfaces.msg import NitrosBridgeImage, NitrosBridgeTensorList
from isaac_ros_pynitros.utils.cpu_shared_mem import CPUSharedMem
from isaac_ros_tensor_list_interfaces.msg import TensorList
import rclpy
from sensor_msgs.msg import Image


class PyNitrosTypeViewBase():
    """
    Base class of PyNITROS message view.

    PyNITROS message view provides functionalities to access the metadata
    and the GPU pointer of the underlying NITROS Bridge message. It can
    also be converted to other data type such as PyTorch tensor or cupy array.
    """

    # Define the syscall numbers for pidfd_open and pidfd_getfd
    SYS_pidfd_open = 434
    SYS_pidfd_getfd = 438

    # Define the syscall function
    syscall = ctypes.CDLL(None).syscall
    syscall.restype = c_long
    syscall.argtypes = [c_long, c_long,
                        c_long, c_long, c_long, c_long]

    nitros_bridge_msg_types = (
        NitrosBridgeImage,
        NitrosBridgeTensorList
    )

    raw_msg_types = (
        Image,
        TensorList
    )

    def __init__(self, raw_msg, gpu_ptr):
        self.gpu_ptr = gpu_ptr
        self.raw_msg = raw_msg
        self.cuda_stream = None
        self.handle = None
        self._cpu_shared_mem = None

    def _open_shm_and_check_uid(self, sender_pid, memblock_fd):
        self._cpu_shared_mem = CPUSharedMem()
        # If UID presents (for backward compatibility),
        # compare UID with it in shared mem and update refcount
        if (self.raw_msg.uid):
            # Create instance of shared cpu mem pool
            self._cpu_shared_mem.open_shm(str(sender_pid) + str(memblock_fd))

            # Acquire lock, check UID, increase refcount if UID match
            self._cpu_shared_mem.lock.acquire()
            memblock_refcount, memblock_uuid = self._cpu_shared_mem.get_value()
            if str(memblock_uuid) != str(self.raw_msg.uid):
                self._cpu_shared_mem.lock.release()
                rclpy.logging.get_logger('PyNITROS Subscriber').info(
                    f'MEMBLOCK UID: {memblock_uuid} != \
                    MSG UID: {self.raw_msg.uid}, dropping message'
                )
                return False
            else:
                self._cpu_shared_mem.update_refcount(1)
                self._cpu_shared_mem.lock.release()
        return True

    def _import_gpu_ptr_from_fd(self, sender_pid, memblock_fd, data_size):
        """Import the shared memory from file descriptor."""
        # FD -> Handle
        pidfd = self._pidfd_open(sender_pid, 0)
        if pidfd == -1:
            raise RuntimeError('pidfd_open failed')

        new_fd = self._pidfd_getfd(pidfd, memblock_fd, 0)
        if new_fd == -1:
            raise RuntimeError('pidfd_getfd failed')

        err, self.handle = cuda.cuMemImportFromShareableHandle(
            new_fd,
            cuda.CUmemAllocationHandleType.CU_MEM_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR)
        self.ASSERT_CUDA_SUCCESS(err)

        # Allocate GPU Space -> CUmemGenericAllocationHandle
        allocProp = cuda.CUmemAllocationProp()
        allocProp.type = cuda.CUmemAllocationType.CU_MEM_ALLOCATION_TYPE_PINNED
        allocProp.location.type = cuda.CUmemLocationType.CU_MEM_LOCATION_TYPE_DEVICE
        allocProp.requestedHandleTypes = \
            cuda.CUmemAllocationHandleType.CU_MEM_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR
        allocProp.location.id = 0

        # Get memory alloc granularity
        err, granularity = cuda.cuMemGetAllocationGranularity(
            allocProp,
            cuda.CUmemAllocationGranularity_flags.CU_MEM_ALLOC_GRANULARITY_MINIMUM)
        self.ASSERT_CUDA_SUCCESS(err)

        rounded_data_size = self._round_up(data_size, granularity)

        # Handle -> Tensor Reconstruction
        res, virtual_ptr = cuda.cuMemAddressReserve(
            rounded_data_size, 0, 0, 0)
        self.ASSERT_CUDA_SUCCESS(res)

        # Map Handle to Virtual Address Range, Set Access
        res, = cuda.cuMemMap(virtual_ptr, rounded_data_size, 0, self.handle, 0)
        self.ASSERT_CUDA_SUCCESS(res)

        accessDesc = cuda.CUmemAccessDesc()
        accessDesc.location = allocProp.location
        accessDesc.flags = cuda.CUmemAccess_flags.CU_MEM_ACCESS_FLAGS_PROT_READWRITE

        res, = cuda.cuMemSetAccess(
            virtual_ptr, rounded_data_size, [accessDesc], 1)
        self.ASSERT_CUDA_SUCCESS(res)

        return virtual_ptr

    def _round_up(self, size, granularity):
        mult = math.ceil(size / float(granularity))
        return mult * granularity

    def _pidfd_open(self, pid, flags):
        return self.syscall(self.SYS_pidfd_open, pid, flags, 0, 0, 0, 0)

    def _pidfd_getfd(self, pidfd, targetfd, flags):
        return self.syscall(self.SYS_pidfd_getfd, pidfd, targetfd, flags, 0, 0, 0)

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

    def set_stream(self, stream):
        """Fill the CUDA stream if async CUDA operations are in the fly."""
        self.cuda_stream = stream

    def postprocess(self):
        """Postprocess the view."""
        if self.cuda_stream:
            err, = cudart.cudaStreamSynchronize(self.cuda_stream)
            self.ASSERT_CUDA_SUCCESS(err)
        if (type(self.raw_msg) in self.nitros_bridge_msg_types):
            # Decrease the refcount
            if (self.raw_msg.uid):
                self._cpu_shared_mem.lock.acquire()
                self._cpu_shared_mem.update_refcount(-1)
                self._cpu_shared_mem.lock.release()
        else:
            # Free the memory
            cudart.cudaFree(self.gpu_ptr)

    def get_handle(self):
        return self.handle

    def get_frame_id(self):
        return self.raw_msg.header.frame_id

    def get_timestamp_seconds(self):
        return self.raw_msg.header.stamp.sec

    def get_timestamp_nanoseconds(self):
        return self.raw_msg.header.stamp.nanosec
