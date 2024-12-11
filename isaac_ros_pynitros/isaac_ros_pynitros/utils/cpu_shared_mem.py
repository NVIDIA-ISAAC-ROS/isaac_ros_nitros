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

import mmap
import struct
import uuid

import posix_ipc


class CPUSharedMem():
    """Shared CPU Mem for tracking CUDA Mem Block Info."""

    def __init__(self):
        self.map_element_size = struct.calcsize('i16s')
        self.cpu_shared_mem = None
        self.cpu_shared_mem_obj = None
        self.lock = None

    def create_shm(self, name):
        """Create a shared memory object with a given name."""
        self.cpu_shared_mem = posix_ipc.SharedMemory(
            name, posix_ipc.O_CREAT, size=self.map_element_size)

        # Attach the shared memory to a byte array
        self.cpu_shared_mem_obj = mmap.mmap(
            self.cpu_shared_mem.fd, self.map_element_size)
        self.lock = posix_ipc.Semaphore(
            name, posix_ipc.O_CREAT, initial_value=1)

    def open_shm(self, name):
        """Open a shared memory object with a given name."""
        self.cpu_shared_mem = posix_ipc.SharedMemory(name)

        # Attach the shared memory to a byte array
        self.cpu_shared_mem_obj = mmap.mmap(
            self.cpu_shared_mem.fd, self.map_element_size)
        self.lock = posix_ipc.Semaphore(name)

    def update_refcount(self, val):
        """Update the reference count of the shared memory."""
        memblock_refcount, memblock_uuid = self.get_value()
        self.put_value((memblock_refcount + val, memblock_uuid))

    def put_value(self, value):
        """Write a value to the shared memory."""
        packed_value = struct.pack('i16s', value[0], value[1].bytes)

        self.cpu_shared_mem_obj.seek(0)
        self.cpu_shared_mem_obj.write(packed_value)
        self.cpu_shared_mem_obj.flush()

    def get_value(self):
        """Read a value from the shared memory."""
        self.cpu_shared_mem_obj.seek(0)
        packed_value = self.cpu_shared_mem_obj.read()

        unpacked_value = struct.unpack('i16s', packed_value)

        # Extract relevant parts of the unpacked value
        memblock_refcount = unpacked_value[0]
        memblock_uuid_bytes = unpacked_value[1]

        # Convert UUID bytes to UUID object
        memblock_uuid = uuid.UUID(bytes=memblock_uuid_bytes)

        return memblock_refcount, memblock_uuid

    def close(self):
        """Close the shared memory object."""
        self.cpu_shared_mem_obj.close()
        self.cpu_shared_mem.close_fd()
        self.cpu_shared_mem.unlink()
        self.lock.unlink()
