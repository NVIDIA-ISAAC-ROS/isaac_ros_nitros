// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>
#include <cuda_runtime.h>

#include <cstring>
#include <functional>
#include <future>
#include <memory>
#include <vector>

#include "isaac_ros_nitros/types/cuda_stream_pool.hpp"
#include "isaac_ros_nitros/types/cuda_memory_pool.hpp"
#include "isaac_ros_nitros/types/nitros_buffer.hpp"
#include "isaac_ros_nitros_image_type/nitros_image.hpp"

namespace nitros = nvidia::isaac_ros::nitros;

constexpr uint32_t kWidth = 4;
constexpr uint32_t kHeight = 4;
constexpr uint32_t kStep = kWidth * 3;
constexpr size_t kBytes = kStep * kHeight;
constexpr char kEncoding[] = "rgb8";

// ---------------------------------------------------------------------------
// Pattern (1): get_read_handle with pool StreamHandle.
//
// Verifies:
//   - Pool stream is usable with NitrosBuffer's read event machinery.
//   - Stream is checked out during the read scope (available decrements).
//   - ReadHandle destructor records its event on the pool stream without error.
//   - Stream is returned to the pool after both handles are destroyed.
//   - Data read through the handle matches what was written.
//
// Setup uses plain cudaMalloc/cudaMemcpy (no pool) so the test is isolated.
// ---------------------------------------------------------------------------
TEST(NitrosImageStreamPool, GetReadHandle)
{
  auto & pool = nitros::CudaStreamPool::instance();
  size_t before = pool.available();

  std::vector<uint8_t> host_src(kBytes, 0xAB);
  std::vector<uint8_t> host_dst(kBytes, 0);

  // Setup: populate image without using the stream pool at all
  nitros::NitrosImage image;
  uint8_t * dptr = nullptr;
  ASSERT_EQ(cudaMalloc(reinterpret_cast<void **>(&dptr), kBytes), cudaSuccess);
  ASSERT_EQ(cudaMemcpy(dptr, host_src.data(), kBytes, cudaMemcpyHostToDevice), cudaSuccess);
  cudaStream_t setup_stream = nullptr;
  ASSERT_EQ(cudaStreamCreate(&setup_stream), cudaSuccess);
  {
    auto write_handle = image.from_external(
      dptr, kBytes, kWidth, kHeight, kStep, kEncoding, setup_stream);
  }
  cudaStreamDestroy(setup_stream);

  EXPECT_EQ(pool.available(), before);

  // Test: consume via get_read_handle with pool StreamHandle
  {
    auto stream_handle = pool.get_stream_handle();
    EXPECT_EQ(pool.available(), before - 1);

    auto read_handle = image.get_read_handle(stream_handle.get());
    ASSERT_NE(read_handle.get_ptr(), nullptr);

    ASSERT_EQ(cudaMemcpyAsync(host_dst.data(), read_handle.get_ptr(), kBytes,
      cudaMemcpyDeviceToHost, stream_handle.get()), cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(stream_handle.get()), cudaSuccess);
  }

  EXPECT_EQ(std::memcmp(host_src.data(), host_dst.data(), kBytes), 0);
  EXPECT_EQ(pool.available(), before);
}

// ---------------------------------------------------------------------------
// Pattern (2): from_pool with pool StreamHandle.
//
// Verifies:
//   - Pool stream is usable with CUDAMemoryPool allocation + NitrosBuffer
//     write event machinery.
//   - WriteHandle destructor records its event on the pool stream without error.
//   - Stream is returned to the pool after both handles are destroyed.
//   - Data written through the handle is readable on the device.
//
// Reads back via synchronous cudaMemcpy (no get_read_handle dependency).
// ---------------------------------------------------------------------------
TEST(NitrosImageStreamPool, FromPool)
{
  auto & pool = nitros::CudaStreamPool::instance();
  size_t before = pool.available();

  nitros::CUDAMemoryPool mem_pool;
  ASSERT_EQ(mem_pool.create(kBytes, 2, nitros::CUDAMemoryPool::MemoryType::Device), cudaSuccess);

  std::vector<uint8_t> host_src(kBytes);
  for (size_t i = 0; i < kBytes; ++i) {
    host_src[i] = static_cast<uint8_t>(i & 0xFF);
  }

  std::vector<uint8_t> host_dst(kBytes, 0);
  nitros::NitrosImage image;
  {
    auto stream_handle = pool.get_stream_handle();
    auto write_handle = image.from_pool(
      mem_pool, kWidth, kHeight, kStep, kEncoding, stream_handle.get());

    ASSERT_NE(write_handle.get_ptr(), nullptr);

    ASSERT_EQ(cudaMemcpyAsync(write_handle.get_ptr(), host_src.data(), kBytes,
      cudaMemcpyHostToDevice, stream_handle.get()), cudaSuccess);

    // Read back directly with synchronous cudaMemcpy (no get_read_handle)
    ASSERT_EQ(cudaStreamSynchronize(stream_handle.get()), cudaSuccess);
    ASSERT_EQ(cudaMemcpy(host_dst.data(), write_handle.get_ptr(), kBytes,
      cudaMemcpyDeviceToHost), cudaSuccess);
  }

  EXPECT_EQ(std::memcmp(host_src.data(), host_dst.data(), kBytes), 0);
  EXPECT_EQ(pool.available(), before);
}

// ---------------------------------------------------------------------------
// Pattern (3): from_external with raw acquire()/release() and async deleter.
//
// Verifies:
//   - Stream stays checked out while the buffer is alive (available decrements).
//   - WriteHandle destructor records its event on the raw stream without error.
//   - Data written through the handle is readable on the device.
//   - After ~NitrosImage, the deleter runs on a background thread and returns
//     the stream to the pool. A std::promise embedded in the deleter lets the
//     test wait deterministically for that to happen.
//
// Reads back via synchronous cudaMemcpy (no get_read_handle dependency).
// ---------------------------------------------------------------------------
TEST(NitrosImageStreamPool, FromExternalDeferredRelease)
{
  auto & pool = nitros::CudaStreamPool::instance();
  size_t before = pool.available();

  std::vector<uint8_t> host_src(kBytes, 0xCD);
  std::vector<uint8_t> host_dst(kBytes, 0);

  auto deleter_done = std::make_shared<std::promise<void>>();
  auto deleter_future = deleter_done->get_future();

  {
    cudaStream_t stream = pool.acquire();
    EXPECT_EQ(pool.available(), before - 1);

    uint8_t * dptr = nullptr;
    ASSERT_EQ(cudaMallocAsync(reinterpret_cast<void **>(&dptr), kBytes, stream), cudaSuccess);

    auto deleter = [&pool, stream, deleter_done](uint8_t * p) {
        cudaFreeAsync(p, stream);
        pool.release(stream);
        deleter_done->set_value();
      };

    nitros::NitrosImage image;
    {
      auto write_handle = image.from_external(
        dptr, kBytes, kWidth, kHeight, kStep, kEncoding, stream, deleter);

      ASSERT_EQ(cudaMemcpyAsync(write_handle.get_ptr(), host_src.data(), kBytes,
        cudaMemcpyHostToDevice, stream), cudaSuccess);
    }

    // Stream is still checked out (held by deleter in image's buffer)
    EXPECT_EQ(pool.available(), before - 1);

    // Read back directly with synchronous cudaMemcpy (no get_read_handle)
    ASSERT_EQ(cudaStreamSynchronize(stream), cudaSuccess);
    ASSERT_EQ(cudaMemcpy(host_dst.data(), dptr, kBytes, cudaMemcpyDeviceToHost), cudaSuccess);

    EXPECT_EQ(std::memcmp(host_src.data(), host_dst.data(), kBytes), 0);
  }
  // ~image runs here -> NitrosBuffer cleanup thread -> deleter -> promise fulfilled

  deleter_future.get();
  EXPECT_EQ(pool.available(), before);
}

// ---------------------------------------------------------------------------
// Stream reuse: sequential from_pool calls get the same stream back.
//
// Verifies:
//   - After a StreamHandle is destroyed and its stream returns to the pool,
//     the next get_stream_handle() reuses that same stream (pointer equality).
//   - Confirms the pool recycles rather than always creating new streams.
// ---------------------------------------------------------------------------
TEST(NitrosImageStreamPool, FromPoolStreamReuse)
{
  auto & pool = nitros::CudaStreamPool::instance();

  nitros::CUDAMemoryPool mem_pool;
  ASSERT_EQ(mem_pool.create(kBytes, 2, nitros::CUDAMemoryPool::MemoryType::Device), cudaSuccess);

  cudaStream_t first_stream = nullptr;
  cudaStream_t second_stream = nullptr;

  {
    nitros::NitrosImage img1;
    auto sh1 = pool.get_stream_handle();
    first_stream = sh1.get();
    auto wh1 = img1.from_pool(mem_pool, kWidth, kHeight, kStep, kEncoding, sh1.get());
  }

  {
    nitros::NitrosImage img2;
    auto sh2 = pool.get_stream_handle();
    second_stream = sh2.get();
    auto wh2 = img2.from_pool(mem_pool, kWidth, kHeight, kStep, kEncoding, sh2.get());
  }

  EXPECT_EQ(first_stream, second_stream);
}
