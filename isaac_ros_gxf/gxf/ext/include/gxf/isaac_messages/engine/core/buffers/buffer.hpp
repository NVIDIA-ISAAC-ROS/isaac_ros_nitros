/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <memory>
#include <type_traits>
#include <utility>

#include "engine/core/allocator/allocators.hpp"
#include "engine/core/buffers/traits.hpp"
#include "engine/core/byte.hpp"

namespace isaac {

// -------------------------------------------------------------------------------------------------

namespace detail {

// A simple pointer which can be tagged. This is used for example to differentiate between a pointer
// to host memory and a pointer to device memory.
template <typename K, typename Tag = byte>
class TaggedPointer {
 public:
  using value_t = std::remove_cv_t<K>;
  using const_pointer_t = TaggedPointer<const value_t, Tag>;
  using pointer_t = TaggedPointer<value_t, Tag>;
  using tag_t = Tag;
  // Standard constructor
  TaggedPointer(K* data = nullptr) : data_(data) {}

  // Default copy
  TaggedPointer(const TaggedPointer& other) = default;
  TaggedPointer& operator=(const TaggedPointer& other) = default;

  // Move will set source to nullptr
  TaggedPointer(TaggedPointer&& other) { *this = std::move(other); }
  TaggedPointer& operator=(TaggedPointer&& other) {
    data_ = other.data_;
    other.data_ = nullptr;
    return *this;
  }

  // Sets the actual pointer
  TaggedPointer& operator=(K* other) {
    data_ = other;
    return *this;
  }
  // Gets the actual pointer
  K* get() const { return data_; }
  operator K*() const { return data_; }

 private:
  K* data_;
};

}  // namespace detail

// -------------------------------------------------------------------------------------------------

namespace detail {

// Base class for Buffer and BufferView which provides storage for the memory pointer
// and corresponding dimensions.
template <typename Pointer>
class BufferBase {
 public:
  using pointer_t = Pointer;

  BufferBase() : pointer_(nullptr), size_(0) {}

  BufferBase(Pointer pointer, size_t size)
      : pointer_(std::move(pointer)), size_(size) {}

  // pointer to the first row
  const Pointer& pointer() const { return pointer_; }
  // The total size of the buffer in bytes
  size_t size() const { return size_; }
  // A pointer to the first byte of the buffer.
  auto begin() const { return pointer_.get(); }
  // A pointer behind the last byte of the buffer.
  auto end() const { return begin() + size(); }

 protected:
  Pointer pointer_;
  size_t size_;
};

}  // namespace detail

// -------------------------------------------------------------------------------------------------

// A buffer which owns its memory
template <typename Pointer, typename Allocator>
class Buffer : public detail::BufferBase<Pointer> {
 public:
  using mutable_view_t = detail::BufferBase<typename Pointer::pointer_t>;
  using const_view_t = detail::BufferBase<typename Pointer::const_pointer_t>;

  Buffer() : handle_(nullptr, Deleter{0}) {}

  // Allocates memory for `size` bytes.
  Buffer(size_t size)
      : detail::BufferBase<Pointer>(nullptr, size),
        handle_(Allocator::Allocate(size), Deleter{size}) {
    // 1) Initialize the base class with a nullptr, the number of rows, and the desired stride.
    // 2) Allocate memory and store it in the unique pointer used as handle. This will also change
    //    the stride stored in the base class to the actual stride chosen by the allocator.
    // 3) Get a pointer to the allocated memory and store it in the base class so that calls to
    //    pointer() actually work.
    this->pointer_ = handle_.get();
  }

  Buffer(Buffer&& buffer)
      : detail::BufferBase<Pointer>(nullptr, buffer.size_),
        handle_(std::move(buffer.handle_)) {
    this->pointer_ = handle_.get();
    buffer.pointer_ = nullptr;
    buffer.size_ = 0;
  }

  Buffer& operator=(Buffer&& buffer) {
    this->handle_ = std::move(buffer.handle_);
    this->pointer_ = this->handle_.get();
    this->size_ = buffer.size_;
    buffer.pointer_ = nullptr;
    buffer.size_ = 0;
    return *this;
  }

  void resize(size_t desired_size) {
    if (desired_size == this->size()) return;
    *this = Buffer<Pointer, Allocator>(desired_size);
  }

  // Disowns the pointer from the buffer. The user is now responsible for deallocation.
  // WARNING: This is dangerous as the wrong allocator might be called.
  byte* release() {
    byte* pointer = handle_.release();
    this->pointer_ = nullptr;
    return pointer;
  }

  // Creates a view which provides read and write access from this buffer object.
  mutable_view_t view() { return mutable_view_t(this->pointer_, this->size_, this->stride_); }
  const_view_t view() const {
    return const_view_t(typename Pointer::const_pointer_t(this->pointer_.get()), this->size_);
  }
  // Creates a view which only provides read access from this buffer object.
  const_view_t const_view() const {
    return const_view_t(typename Pointer::const_pointer_t(this->pointer_.get()), this->size_);
  }

 private:
  // A deleter object for the unique pointer to free allocated memory
  struct Deleter {
    size_t size;
    void operator()(byte* pointer) {
      if (size == 0) return;
      Allocator::Deallocate(pointer, size);
    }
  };

  // A unique pointer is used to handle the allocator and to make this object non-copyable.
  using handle_t = std::unique_ptr<byte, Deleter>;

  // Memory handle used to automatically deallocate memory and to disallow copy semantics.
  handle_t handle_;
};

// -------------------------------------------------------------------------------------------------

// A pointer to host memory
template <typename K>
using HostPointer =
    detail::TaggedPointer<K, std::integral_constant<BufferStorageMode, BufferStorageMode::Host>>;

// A host buffer with stride which owns its memory
using CpuBuffer = Buffer<HostPointer<byte>, CpuAllocator>;

// A host buffer with stride which does not own its memory
using CpuBufferView = detail::BufferBase<HostPointer<byte>>;

// A host buffer with stride which does not own its memory and provides only read access
using CpuBufferConstView = detail::BufferBase<HostPointer<const byte>>;

template <>
struct BufferTraits<Buffer<HostPointer<byte>, CpuAllocator>> {
  static constexpr BufferStorageMode kStorageMode = BufferStorageMode::Host;
  static constexpr bool kIsMutable = true;
  static constexpr bool kIsOwning = true;

  using buffer_view_t = CpuBufferView;
  using buffer_const_view_t = CpuBufferConstView;
};

template <typename K>
struct BufferTraits<detail::BufferBase<HostPointer<K>>> {
  static constexpr BufferStorageMode kStorageMode = BufferStorageMode::Host;
  static constexpr bool kIsMutable = !std::is_const<K>::value;
  static constexpr bool kIsOwning = false;

  using buffer_view_t = CpuBufferView;
  using buffer_const_view_t = CpuBufferConstView;
};

// -------------------------------------------------------------------------------------------------

// A pointer to CUDA memory
template <typename K>
using CudaPointer =
    detail::TaggedPointer<K, std::integral_constant<BufferStorageMode, BufferStorageMode::Cuda>>;

// A CUDA buffer with stride which owns its memory
using CudaBuffer = Buffer<CudaPointer<byte>, CudaAllocator>;

// A CUDA buffer with stride which does not own its memory
using CudaBufferView = detail::BufferBase<CudaPointer<byte>>;

// A CUDA buffer with stride which does not own its memory and provides only read access
using CudaBufferConstView = detail::BufferBase<CudaPointer<const byte>>;

template <>
struct BufferTraits<Buffer<CudaPointer<byte>, CudaAllocator>> {
  static constexpr BufferStorageMode kStorageMode = BufferStorageMode::Cuda;
  static constexpr bool kIsMutable = true;
  static constexpr bool kIsOwning = true;

  using buffer_view_t = CudaBufferView;
  using buffer_const_view_t = CudaBufferConstView;
};

template <typename K>
struct BufferTraits<detail::BufferBase<CudaPointer<K>>> {
  static constexpr BufferStorageMode kStorageMode = BufferStorageMode::Cuda;
  static constexpr bool kIsMutable = !std::is_const<K>::value;
  static constexpr bool kIsOwning = false;

  using buffer_view_t = CudaBufferView;
  using buffer_const_view_t = CudaBufferConstView;
};

// -------------------------------------------------------------------------------------------------

}  // namespace isaac
