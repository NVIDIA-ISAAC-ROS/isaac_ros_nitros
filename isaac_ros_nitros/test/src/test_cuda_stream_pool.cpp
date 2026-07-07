// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <atomic>
#include <set>
#include <thread>
#include <vector>
#include <functional>

#include "isaac_ros_nitros/types/cuda_stream_pool.hpp"

namespace nitros = nvidia::isaac_ros::nitros;

TEST(CudaStreamPool, SingletonReturnsSameInstance)
{
  auto & a = nitros::CudaStreamPool::instance();
  auto & b = nitros::CudaStreamPool::instance();
  EXPECT_EQ(&a, &b);
}

TEST(CudaStreamPool, AcquiredStreamIsValid)
{
  nitros::CudaStreamPool pool(2);
  cudaStream_t s = pool.acquire();
  ASSERT_NE(s, nullptr);
  cudaError_t err = cudaStreamQuery(s);
  EXPECT_TRUE(err == cudaSuccess || err == cudaErrorNotReady);
  pool.release(s);
}

TEST(CudaStreamPool, StreamIsReusedAfterRelease)
{
  nitros::CudaStreamPool pool(1);
  cudaStream_t first = pool.acquire();
  pool.release(first);
  cudaStream_t second = pool.acquire();
  EXPECT_EQ(first, second);
  pool.release(second);
}

TEST(CudaStreamPool, OverflowCreatesNewStream)
{
  nitros::CudaStreamPool pool(2);
  cudaStream_t s1 = pool.acquire();
  cudaStream_t s2 = pool.acquire();
  cudaStream_t s3 = pool.acquire();
  ASSERT_NE(s3, nullptr);
  EXPECT_NE(s3, s1);
  EXPECT_NE(s3, s2);
  pool.release(s1);
  pool.release(s2);
  pool.release(s3);
}

TEST(CudaStreamPool, OverflowStreamReturnedToPool)
{
  nitros::CudaStreamPool pool(1);
  cudaStream_t s1 = pool.acquire();
  cudaStream_t s2 = pool.acquire();
  pool.release(s1);
  pool.release(s2);
  EXPECT_EQ(pool.available(), 2u);
  cudaStream_t a = pool.acquire();
  cudaStream_t b = pool.acquire();
  EXPECT_NE(a, nullptr);
  EXPECT_NE(b, nullptr);
  pool.release(a);
  pool.release(b);
}

TEST(CudaStreamPool, ReleaseNullIsNoop)
{
  nitros::CudaStreamPool pool(2);
  size_t before = pool.available();
  pool.release(nullptr);
  EXPECT_EQ(pool.available(), before);
}

TEST(CudaStreamPool, ConcurrentAcquireRelease)
{
  nitros::CudaStreamPool pool(4);
  constexpr int kThreads = 8;
  constexpr int kIterations = 500;
  std::atomic<int> errors{0};

  auto worker = [&]() {
      for (int i = 0; i < kIterations; ++i) {
        cudaStream_t s = pool.acquire();
        if (s == nullptr) {
          errors++;
          continue;
        }
        cudaError_t err = cudaStreamQuery(s);
        if (err != cudaSuccess && err != cudaErrorNotReady) {
          errors++;
        }
        pool.release(s);
      }
    };

  std::vector<std::thread> threads;
  threads.reserve(kThreads);
  for (int i = 0; i < kThreads; ++i) {
    threads.emplace_back(worker);
  }
  for (auto & t : threads) {
    t.join();
  }

  EXPECT_EQ(errors.load(), 0);
}

TEST(CudaStreamPool, NoDuplicatesConcurrently)
{
  constexpr int kThreads = 8;
  nitros::CudaStreamPool pool(kThreads);
  std::vector<cudaStream_t> acquired(kThreads, nullptr);
  std::mutex barrier_mutex;
  std::condition_variable barrier_cv;
  int arrived = 0;

  auto worker = [&](int idx) {
      acquired[idx] = pool.acquire();
      {
        std::unique_lock<std::mutex> lk(barrier_mutex);
        arrived++;
        if (arrived == kThreads) {
          barrier_cv.notify_all();
        } else {
          barrier_cv.wait(lk, [&]() {return arrived == kThreads;});
        }
      }
      pool.release(acquired[idx]);
    };

  std::vector<std::thread> threads;
  threads.reserve(kThreads);
  for (int i = 0; i < kThreads; ++i) {
    threads.emplace_back(worker, i);
  }
  for (auto & t : threads) {
    t.join();
  }

  std::set<cudaStream_t> unique(acquired.begin(), acquired.end());
  EXPECT_EQ(unique.size(), static_cast<size_t>(kThreads));
}

TEST(CudaStreamPool, DeferredReleasePattern)
{
  nitros::CudaStreamPool pool(2);
  cudaStream_t s = pool.acquire();

  std::function<void()> deferred_release = [&pool, s]() {
      pool.release(s);
    };

  void * dptr = nullptr;
  ASSERT_EQ(cudaMallocAsync(&dptr, 1024, s), cudaSuccess);
  ASSERT_EQ(cudaFreeAsync(dptr, s), cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(s), cudaSuccess);

  deferred_release();

  cudaStream_t reused = pool.acquire();
  EXPECT_EQ(reused, s);
  pool.release(reused);
}

TEST(CudaStreamPool, AvailableCountTracksAcquireRelease)
{
  nitros::CudaStreamPool pool(3);
  EXPECT_EQ(pool.available(), 3u);
  cudaStream_t s1 = pool.acquire();
  EXPECT_EQ(pool.available(), 2u);
  cudaStream_t s2 = pool.acquire();
  EXPECT_EQ(pool.available(), 1u);
  pool.release(s1);
  EXPECT_EQ(pool.available(), 2u);
  pool.release(s2);
  EXPECT_EQ(pool.available(), 3u);
}

TEST(CudaStreamPool, StreamHandleReturnsValidStream)
{
  nitros::CudaStreamPool pool(2);
  auto handle = pool.get_stream_handle();
  EXPECT_NE(handle.get(), nullptr);
  cudaError_t err = cudaStreamQuery(handle.get());
  EXPECT_TRUE(err == cudaSuccess || err == cudaErrorNotReady);
}

TEST(CudaStreamPool, StreamHandleAutoReleasesOnDestruction)
{
  nitros::CudaStreamPool pool(2);
  EXPECT_EQ(pool.available(), 2u);
  {
    auto handle = pool.get_stream_handle();
    EXPECT_EQ(pool.available(), 1u);
    (void)handle;
  }
  EXPECT_EQ(pool.available(), 2u);
}

TEST(CudaStreamPool, StreamHandleAutoReleasesOnException)
{
  nitros::CudaStreamPool pool(2);
  EXPECT_EQ(pool.available(), 2u);
  try {
    auto handle = pool.get_stream_handle();
    EXPECT_EQ(pool.available(), 1u);
    throw std::runtime_error("test");
  } catch (...) {
  }
  EXPECT_EQ(pool.available(), 2u);
}

TEST(CudaStreamPool, StreamHandleMoveConstructTransfersOwnership)
{
  nitros::CudaStreamPool pool(2);
  auto handle1 = pool.get_stream_handle();
  cudaStream_t s = handle1.get();
  EXPECT_EQ(pool.available(), 1u);
  nitros::CudaStreamPool::StreamHandle handle2(std::move(handle1));
  EXPECT_EQ(handle2.get(), s);
  EXPECT_EQ(handle1.get(), nullptr);
  EXPECT_EQ(pool.available(), 1u);
}
