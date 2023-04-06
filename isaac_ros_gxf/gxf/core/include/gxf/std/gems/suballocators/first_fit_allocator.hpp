/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#ifndef NVIDIA_GXF_STD_GEMS_SUBALLOCATORS_FIRST_FIT_ALLOCATOR_HPP
#define NVIDIA_GXF_STD_GEMS_SUBALLOCATORS_FIRST_FIT_ALLOCATOR_HPP

#include <cstdint>
#include <memory>
#include <new>
#include <utility>

#include "common/expected.hpp"
#include "gxf/std/gems/suballocators/first_fit_allocator_base.hpp"

namespace nvidia {
namespace gxf {

// Memory management class.
// It pre-allocates memory for a given type and allow acquiring chunk of memory and releasing them.
// Only the allocate function is doing some memory allocation, other functions are using constant
// amount of memory on the stack.
template<class T>
class FirstFitAllocator {
 public:
  // Expected type used by this class.
  template <typename E>
  using expected_t = nvidia::Expected<E, FirstFitAllocatorBase::Error>;
  // Unexpected type used by this class.
  using unexpected_t = nvidia::Unexpected<FirstFitAllocatorBase::Error>;

  FirstFitAllocator() = default;

  // Allocates the require memory to handle the query for a chunk of memory of a give size .
  // It fails if size is invalid (too big or negative).
  // Note that a minimum chunk size to be considered can be provided. It means the memory allocated
  // will always be a multiple of this size. This can help speed up the memory allocation, however
  // it loses control on the exact allocated size.
  // The total memory allocated will be around sizeof(T) * size + 32 * size / chunk_size.
  expected_t<int32_t> allocate(const int32_t size, const int chunk_size = 1) {
    if (buffer_.get() != nullptr) {
      return unexpected_t(FirstFitAllocatorBase::Error::kAlreadyInUse);
    }
    if (size < 0 || chunk_size <= 0) {
      return unexpected_t(FirstFitAllocatorBase::Error::kInvalidSize);
    }
    chunk_size_ = chunk_size;
    const int32_t number_of_chunks = getNumberOfChunks(size);
    // Allocate memory
    buffer_.reset(new(std::nothrow) T[number_of_chunks * chunk_size_]);
    if (buffer_.get() == nullptr) {
      return unexpected_t(FirstFitAllocatorBase::Error::kOutOfMemory);
    }
    // Prepare the memory management.
    auto res = memory_management_.allocate(number_of_chunks);
    if (!res) {
      return unexpected_t(res.error());
    }
    return size;
  }

  // Attempts to acquire a block of memory of a given size.
  // If such a contiguous block exists, it will return a pointer to that block and the actual size
  // acquired (will be the smallest multiple of chunk_size_ that exceed or equal size).
  // If no block exists, it will return FirstFitAllocatorBase::Error::kOutOfMemory.
  expected_t<std::pair<T*, int32_t>> acquire(const int32_t size) {
    const int32_t number_of_chunks = getNumberOfChunks(size);
    auto res = memory_management_.acquire(number_of_chunks);
    if (!res) {
      return unexpected_t(res.error());
    }
    return std::make_pair(&buffer_.get()[res.value() * chunk_size_],
                          number_of_chunks * chunk_size_);
  }

  // Releases a block of memory that has been acquired with the function above.
  // If there was no `acquire(size)` that returned this pointer, this query will fail, otherwise it
  // will free the block of memory for further acquisition.
  // Note once a block of memory has been released, it can't be released again.
  expected_t<void> release(const T* ptr) {
    const int32_t index = std::distance<const T*>(buffer_.get(), ptr);
    if (index % chunk_size_ != 0) {
      return unexpected_t(FirstFitAllocatorBase::Error::kInvalidSize);
    }
    return memory_management_.release(index / chunk_size_);
  }

 private:
  // Returns the number of chunks of memory needed to have at least a given size.
  int32_t getNumberOfChunks(const int32_t size) const {
    return (size + chunk_size_ - 1) / chunk_size_;
  }

  // Real memory management.
  FirstFitAllocatorBase memory_management_;
  // Size of the chunk of memory. Only multiple of this size can be allocated. Whenever a request is
  // made, the allocated size will be the smaller multiple of chunk_size_ that is not smaller than
  // the requested size.
  int32_t chunk_size_;
  // Buffer holding the pre-allocated memory that is provided on demand.
  std::unique_ptr<T[]> buffer_;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_STD_GEMS_SUBALLOCATORS_FIRST_FIT_ALLOCATOR_HPP
