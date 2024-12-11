// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2021-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#ifndef NVIDIA_GXF_COMMON_FIXED_VECTOR_HPP_
#define NVIDIA_GXF_COMMON_FIXED_VECTOR_HPP_

#include <cstring>
#include <utility>

#include "common/byte.hpp"
#include "common/expected.hpp"
#include "common/iterator.hpp"
#include "common/memory_utils.hpp"

namespace nvidia {

// Data structure that provides similar functionality to std::vector but does not dynamically
// reallocate memory and uses Expected type for error handling instead of exceptions.
// This container is not thread-safe.
//
// This container supports allocating memory on the stack or on the heap. If the template argument
// N is specified and greater than 0, a container with a capacity of N will be created on the stack.
// The stack allocated container cannot be resized after it is initialized. The heap allocated
// container must use the reserve function to allocate memory before use. It does not support
// copy assignment or construction.

// Base implementation
template <typename T>
class FixedVectorBase {
 public:
  // Use STL naming convention for compatibility with STL algorithms
  using value_type             = T;
  using size_type              = size_t;
  using iterator               = RandomAccessIterator<FixedVectorBase>;
  using reverse_iterator       = ReverseIterator<iterator>;
  using const_iterator         = ConstRandomAccessIterator<FixedVectorBase>;
  using const_reverse_iterator = ReverseIterator<const_iterator>;

  virtual ~FixedVectorBase() = default;

  // Custom error codes for vector
  enum struct Error {
    kOutOfMemory,         // Memory allocation failed
    kArgumentOutOfRange,  // Argument is out of valid range
    kContainerEmpty,      // Container is empty
    kContainerFull,       // Container is fixed and reached max capacity
    kInvalidIterator,     // Iterator is invalid
  };

  // Expected type which uses class specific errors
  template <typename U>
  using Expected = Expected<U, Error>;

  constexpr bool operator==(const FixedVectorBase& other) const {
    if (size_ != other.size_) {
      return false;
    }
    if (data_ == other.data_) {
      return true;
    }
    for (size_t i = 0; i < size_; i ++) {
      if (data_[i] != other.data_[i]) {
        return false;
      }
    }
    return true;
  }
  constexpr bool operator!=(const FixedVectorBase& other) const { return !(*this == other); }

  constexpr iterator         begin()  { return iterator(*this, 0); }
  constexpr iterator         end()    { return iterator(*this, size_); }
  constexpr reverse_iterator rbegin() { return reverse_iterator(end()); }
  constexpr reverse_iterator rend()   { return reverse_iterator(begin()); }

  constexpr const_iterator         begin()  const { return cbegin(); }
  constexpr const_iterator         end()    const { return cend(); }
  constexpr const_reverse_iterator rbegin() const { return crbegin(); }
  constexpr const_reverse_iterator rend()   const { return crend(); }

  constexpr const_iterator         cbegin()  const { return const_iterator(*this, 0); }
  constexpr const_iterator         cend()    const { return const_iterator(*this, size_); }
  constexpr const_reverse_iterator crbegin() const { return const_reverse_iterator(cend()); }
  constexpr const_reverse_iterator crend()   const { return const_reverse_iterator(cbegin()); }

  constexpr Expected<T&> operator[](size_t index) { return at(index); }
  constexpr Expected<const T&> operator[](size_t index) const { return at(index); }

  // Returns a pointer to data
  constexpr T* data() { return data_; }
  // Returns a read-only pointer to data
  constexpr const T* data() const { return data_; }
  // Returns the number of elements the vector can currently hold
  constexpr size_t capacity() const { return capacity_; }
  // Returns the number of elements in the vector
  constexpr size_t size() const { return size_; }
  // Returns true if the vector contains no elements
  constexpr bool empty() const { return size_ == 0; }
  // Returns true if the vector reached capacity
  constexpr bool full() const { return size_ == capacity_; }

  // Returns a reference to the element at the given index
  constexpr Expected<T&> at(size_t index) {
    if (index >= size_) {
      return Unexpected<Error>{Error::kArgumentOutOfRange};
    }
    return data_[index];
  }

  // Returns a read-only reference to the element at the given index
  constexpr Expected<const T&> at(size_t index) const {
    if (index >= size_) {
      return Unexpected<Error>{Error::kArgumentOutOfRange};
    }
    return data_[index];
  }

  // Returns a reference to the first element
  constexpr Expected<T&> front() {
    if (empty()) {
      return Unexpected<Error>{Error::kContainerEmpty};
    }
    return data_[0];
  }

  // Returns a read-only reference to the first element
  constexpr Expected<const T&> front() const {
    if (empty()) {
      return Unexpected<Error>{Error::kContainerEmpty};
    }
    return data_[0];
  }

  // Returns a reference to the last element
  constexpr Expected<T&> back() {
    if (empty()) {
      return Unexpected<Error>{Error::kContainerEmpty};
    }
    return data_[size_ - 1];
  }

  // Returns a read-only reference to the last element
  constexpr Expected<const T&> back() const {
    if (empty()) {
      return Unexpected<Error>{Error::kContainerEmpty};
    }
    return data_[size_ - 1];
  }

  // Creates a new object with the provided arguments and adds it at the specified index
  template <typename... Args>
  constexpr Expected<void> emplace(size_t index, Args&&... args) {
    if (index > size_) {
      return Unexpected<Error>{Error::kArgumentOutOfRange};
    }
    if (full()) {
      return Unexpected<Error>{Error::kContainerFull};
    }
    if (index < size_) {
      ArrayMoveConstruct(BytePointer(&data_[index + 1]), &data_[index], size_ - index);
    }
    InplaceConstruct<T>(BytePointer(&data_[index]), std::forward<Args>(args)...);
    size_++;
    return kSuccess;
  }

  // Creates a new object with the provided arguments and adds it to the end of the vector
  template <typename... Args>
  constexpr Expected<void> emplace_back(Args&&... args) {
    return emplace(size_, std::forward<Args>(args)...);
  }

  // Copies the object to the specified index
  constexpr Expected<void> insert(size_t index, const T& obj) { return emplace(index, obj); }
  // Moves the object to the specified index
  constexpr Expected<void> insert(size_t index, T&& obj) {
    return emplace(index, std::forward<T>(obj));
  }

  // Insert elements from another fixed vector object to the specified index
  constexpr Expected<void> insert(iterator index, iterator start, iterator end) {
        size_t count = static_cast<size_t>(std::distance(start, end));
        size_t pos = static_cast<size_t>(std::distance(begin(), index));
        if ((pos > size_) || (pos + count > capacity_) || (count < 0)) {
            return Unexpected<Error>{Error::kArgumentOutOfRange};
        }

        auto maybe_value = *start;
        if (maybe_value) {
          T* ptr = &(*maybe_value);
          ArrayCopyConstruct(BytePointer(data_ + pos), ptr, count);
          size_ += count;
        } else {
          return Unexpected<Error>{Error::kInvalidIterator};
        }

        return kSuccess;
  }

  // Copies the object to the end of the vector
  constexpr Expected<void> push_back(const T& obj) { return emplace_back(obj); }
  // Moves the object to the end of the vector
  constexpr Expected<void> push_back(T&& obj) { return emplace_back(std::forward<T>(obj)); }

  // Removes the object at the specified index and destroys it
  constexpr Expected<void> erase(size_t index) {
    if (index >= size_) {
      return Unexpected<Error>{Error::kArgumentOutOfRange};
    }
    if (empty()) {
      return Unexpected<Error>{Error::kContainerEmpty};
    }
    Destruct<T>(BytePointer(&data_[index]));
    size_--;
    if (index < size_) {
      ArrayMoveConstruct(BytePointer(&data_[index]), &data_[index + 1], size_ - index);
    }
    return kSuccess;
  }

  // Removes the object at the end of the vector and destroys it
  constexpr Expected<void> pop_back() { return erase(size_ - 1); }

  // Removes all objects from the vector
  constexpr void clear() {
    while (size_ > 0) {
      Destruct<T>(BytePointer(&data_[--size_]));
    }
  }

  // Resizes the vector by removing objects from the end if shrinking
  // or by adding default objects to the end if expanding
  constexpr Expected<void> resize(size_t count) {
    if (count > capacity_) {
      return Unexpected<Error>{Error::kArgumentOutOfRange};
    }
    while (count > size_) {
      push_back(T());
    }
    while (count < size_) {
      pop_back();
    }
    return kSuccess;
  }

  // Resizes the vector by removing objects from the end if shrinking
  // or by adding copies of the given object to the end if expanding
  constexpr Expected<void> resize(size_t count, const T& obj) {
    if (count > capacity_) {
      return Unexpected<Error>{Error::kArgumentOutOfRange};
    }
    while (count > size_) {
      push_back(obj);
    }
    while (count < size_) {
      pop_back();
    }
    return kSuccess;
  }

 protected:
  // Special value for returning a success
  const Expected<void> kSuccess{};

  FixedVectorBase() : data_{nullptr}, capacity_{0}, size_{0} {};

  // Pointer to an array of objects
  T* data_;
  // Maximum number of objects the container can hold
  size_t capacity_;
  // Number of objects stored
  size_t size_;
};

// Special value used to instantiate a FixedVector with heap memory allocation
constexpr ssize_t kFixedVectorHeap = -1;

// Vector with stack memory allocation
template <typename T, ssize_t N = kFixedVectorHeap>
class FixedVector : public FixedVectorBase<T> {
 public:
  static_assert(N >= 0, "N must be non-negative");

  constexpr FixedVector() {
    data_ = ValuePointer<T>(pool_);
    capacity_ = N;
  }
  constexpr FixedVector(const FixedVector& other) { *this = other; }
  constexpr FixedVector(FixedVector&& other) { *this = std::move(other); }
  constexpr FixedVector& operator=(const FixedVector& other) {
    if (this != &other) {
      data_ = ValuePointer<T>(pool_);
      capacity_ = N;
      ArrayCopyConstruct(BytePointer(data_), other.data_, other.size_);
      size_ = other.size_;
    }
    return *this;
  }
  constexpr FixedVector& operator=(FixedVector&& other) {
    if (this != &other) {
      data_ = ValuePointer<T>(pool_);
      capacity_ = N;
      ArrayMoveConstruct(BytePointer(data_), other.data_, other.size_);
      std::swap(size_, other.size_);
    }
    return *this;
  }
  ~FixedVector() { FixedVectorBase<T>::clear(); }

 private:
  using FixedVectorBase<T>::data_;
  using FixedVectorBase<T>::capacity_;
  using FixedVectorBase<T>::size_;

  // Size of memory pool for stack allocation
  static constexpr size_t kPoolSize = N * sizeof(T);
  // Memory pool for storing objects on the stack
  byte pool_[kPoolSize];
};

// Vector with heap memory allocation
template <typename T>
class FixedVector<T, kFixedVectorHeap> : public FixedVectorBase<T> {
 public:
  template<typename U>
  using Expected = typename FixedVectorBase<T>::template Expected<U>;

  constexpr FixedVector() = default;
  constexpr FixedVector(const FixedVector& other) = delete;
  constexpr FixedVector(FixedVector&& other) { *this = std::move(other); }
  constexpr FixedVector& operator=(const FixedVector& other) = delete;
  constexpr FixedVector& operator=(FixedVector&& other) {
    if (this != &other) {
      std::swap(data_, other.data_);
      std::swap(capacity_, other.capacity_);
      std::swap(size_, other.size_);
    }
    return *this;
  }
  ~FixedVector() {
    FixedVectorBase<T>::clear();
    DeallocateArray<T>(data_);
  }

  // Copies the contents of the given vector
  // Current contents are discarded
  // Fails if given vector is larger than current capacity
  constexpr Expected<void> copy_from(const FixedVector& other) {
    if (other.size_ > capacity_) {
      return Unexpected<Error>{Error::kArgumentOutOfRange};
    }
    FixedVectorBase<T>::clear();
    size_ = other.size_;
    ArrayCopyConstruct(BytePointer(data_), other.data_, size_);
    return kSuccess;
  }

  // Allocates memory to hold the specified number of elements
  constexpr Expected<void> reserve(size_t capacity) {
    if (capacity > capacity_) {
      T* data = AllocateArray<T>(capacity);
      if (!data) {
        return Unexpected<Error>{Error::kOutOfMemory};
      }
      ArrayMoveConstruct(BytePointer(data), data_, size_);
      DeallocateArray<T>(data_);
      data_ = data;
      capacity_ = capacity;
    }
    return kSuccess;
  }

  // Shrinks memory allocation to fit current number of elements
  constexpr Expected<void> shrink_to_fit() {
    if (size_ < capacity_) {
      T* data = AllocateArray<T>(size_);
      if (!data) {
        return Unexpected<Error>(Error::kOutOfMemory);
      }
      ArrayMoveConstruct(BytePointer(data), data_, size_);
      DeallocateArray<T>(data_);
      data_ = data;
      capacity_ = size_;
    }
    return kSuccess;
  }

 private:
  using Error = typename FixedVectorBase<T>::Error;
  using FixedVectorBase<T>::kSuccess;
  using FixedVectorBase<T>::data_;
  using FixedVectorBase<T>::capacity_;
  using FixedVectorBase<T>::size_;
};

}  // namespace nvidia

#endif  // NVIDIA_GXF_COMMON_FIXED_VECTOR_HPP_
