/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <type_traits>

namespace isaac {

// Specifies where data is stored, e.g. host or CUDA device
enum class BufferStorageMode {
  // host memory
  Host,
  // CUDA memory
  Cuda
};

// Specifies various properties for buffer types.
// Example:
//    template <typename T>
//    struct BufferTraits<CudaArray<T>> {
//      static constexpr BufferStorageMode kStorageMode = BufferStorageMode::Cuda;
//      static constexpr bool kIsMutable = true;
//      static constexpr bool kIsOwning = true;
//
//      static CudaArray<T> Create(size_t size) {
//        return CudaArray<T>(size);
//      }
//    };
template <typename Container>
struct BufferTraits;

}  // namespace isaac
