// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#ifndef NVIDIA_GXF_COMMON_FIXED_MAP_HPP_
#define NVIDIA_GXF_COMMON_FIXED_MAP_HPP_

#include <iterator>
#include <utility>

#include "common/expected.hpp"
#include "common/memory_utils.hpp"
#include "common/type_utils.hpp"

namespace nvidia {

// Data structure that provides similar functionality to std::unordered_map but does not dynamically
// reallocate memory and uses Expected type for error handling instead of exceptions.
// This container is not thread-safe.
template <typename Key, typename T, typename Hash = std::hash<Key>>
class FixedMap {
 public:
  template <typename TContainer, typename TValue = typename TContainer::value_type>
  class Iterator;
  template <typename TIterator>
  class ReverseIterator;
  template <typename TContainer>
  using ConstIterator = Iterator<const TContainer, const typename TContainer::value_type>;

  // Use STL naming convension for compatibility with STL algorithms
  using key_type               = Key;
  using mapped_type            = T;
  using value_type             = std::pair<const key_type, mapped_type>;
  using size_type              = size_t;
  using iterator               = Iterator<FixedMap>;
  using reverse_iterator       = ReverseIterator<iterator>;
  using const_iterator         = ConstIterator<FixedMap>;
  using const_reverse_iterator = ReverseIterator<const_iterator>;

  // Custom error codes for map
  enum struct Error {
    kOutOfMemory,      // Memory allocation failed
    kInvalidKey,       // Key is not in container
    kDuplicateKey,     // Key is already in container
    kContainerFull,    // Container is full
    kInvalidIterator,  // Iterator is invalid
  };

  // Expected type which uses class specific errors
  template <typename U>
  using Expected = Expected<U, Error>;

  template <typename TContainer, typename TValue>
  class Iterator : public std::iterator<std::bidirectional_iterator_tag, TValue> {
   public:
    static_assert(IsSame_v<RemoveConst_t<TContainer>, FixedMap>);

    using typename std::iterator<std::bidirectional_iterator_tag, TValue>::difference_type;

    constexpr Iterator() : container_{nullptr}, index_{-1} {}
    constexpr Iterator(TContainer& container, size_t start) : container_{&container}, index_{0} {
      *this += start;
    }

    constexpr Iterator(const Iterator& other) = default;
    constexpr Iterator(Iterator&& other) = default;
    constexpr Iterator& operator=(const Iterator& other) = default;
    constexpr Iterator& operator=(Iterator&& other) = default;

    constexpr Expected<TValue&> operator*() const {
      if (container_ == nullptr || container_->data_ == nullptr ||
          index_ < 0 || index_ >= static_cast<difference_type>(container_->capacity_)) {
        return Unexpected<Error>{Error::kInvalidIterator};
      }
      Bucket& bucket = container_->data_[index_];
      if (!bucket.occupied) {
        return Unexpected<Error>{Error::kInvalidIterator};
      }
      return bucket.value;
    }

    constexpr Iterator& operator+=(difference_type offset) {
      if (container_ == nullptr || container_->data_ == nullptr) {
        return *this;
      }

      while (true) {
        const Bucket& bucket = container_->data_[index_];
        if (bucket.occupied) {
          break;
        }
        index_++;
        if (index_ >= static_cast<difference_type>(container_->capacity_)) {
          break;
        }
      }

      if ((offset > 0 && index_ >= static_cast<difference_type>(container_->capacity_)) ||
          (offset < 0 && index_ <= 0)) {
        return *this;
      }

      while (offset > 0) {
        index_++;
        if (index_ >= static_cast<difference_type>(container_->capacity_)) {
          break;
        }
        const Bucket& bucket = container_->data_[index_];
        if (bucket.occupied) {
          offset--;
        }
      }

      while (offset < 0) {
        index_--;
        if (index_ <= 0) {
          break;
        }
        const Bucket& bucket = container_->data_[index_];
        if (bucket.occupied) {
          offset++;
        }
      }

      return *this;
    }
    constexpr Iterator& operator++() {
      *this += 1;
      return *this;
    }
    constexpr Iterator operator++(int) {
      Iterator iter = *this;
      ++(*this);
      return iter;
    }
    constexpr Iterator& operator-=(difference_type offset) {
      *this += -offset;
      return *this;
    }
    constexpr Iterator& operator--() {
      *this -= 1;
      return *this;
    }
    constexpr Iterator operator--(int) {
      Iterator iter = *this;
      --(*this);
      return iter;
    }

    friend constexpr Iterator operator+(Iterator a, difference_type n) {
      a += n;
      return a;
    }
    friend constexpr Iterator operator+(difference_type n, Iterator a) {
      return a + n;
    }
    friend constexpr Iterator operator-(Iterator a, difference_type n) {
      a -= n;
      return a;
    }
    friend constexpr difference_type operator-(const Iterator& a, const Iterator& b) {
      return a.index_ - b.index_;
    }
    friend constexpr bool operator==(const Iterator& a, const Iterator& b) {
      return a.container_ == b.container_ && a.index_ == b.index_;
    }
    friend constexpr bool operator!=(const Iterator& a, const Iterator& b) {
      return !(a == b);
    }

   private:
    // Container pointer
    TContainer* container_;
    // Iterator index
    difference_type index_;
  };

  template <typename TIterator>
  class ReverseIterator
      : public std::iterator<std::bidirectional_iterator_tag, typename TIterator::value_type> {
   public:
    static_assert(IsSame_v<TIterator, iterator> || IsSame_v<TIterator, const_iterator>);

    using difference_type = typename TIterator::difference_type;
    using TValue = typename TIterator::value_type;

    constexpr explicit ReverseIterator() : iter_{} {}
    constexpr explicit ReverseIterator(TIterator iter) : iter_{iter} {}

    constexpr ReverseIterator(const ReverseIterator& other) = default;
    constexpr ReverseIterator(ReverseIterator&& other) = default;
    constexpr ReverseIterator& operator=(const ReverseIterator& other) = default;
    constexpr ReverseIterator& operator=(ReverseIterator&& other) = default;

    constexpr TIterator base() const { return iter_; }

    constexpr Expected<TValue&> operator*() const { return *std::prev(iter_); }

    constexpr ReverseIterator& operator+=(difference_type offset) {
      iter_ -= offset;
      return *this;
    }
    constexpr ReverseIterator& operator++() {
      iter_ -= 1;
      return *this;
    }
    constexpr ReverseIterator operator++(int) {
      ReverseIterator iter = *this;
      ++(*this);
      return iter;
    }
    constexpr ReverseIterator& operator-=(difference_type offset) {
      iter_ += offset;
      return *this;
    }
    constexpr ReverseIterator& operator--() {
      iter_ += 1;
      return *this;
    }
    constexpr ReverseIterator operator--(int) {
      ReverseIterator iter = *this;
      --(*this);
      return iter;
    }

    friend constexpr ReverseIterator operator+(ReverseIterator a, difference_type n) {
      return ReverseIterator(a.iter_ - n);
    }
    friend constexpr ReverseIterator operator+(difference_type n, ReverseIterator a) {
      return ReverseIterator(a.iter_ - n);
    }
    friend constexpr ReverseIterator operator-(ReverseIterator a, difference_type n) {
      return ReverseIterator(a.iter_ + n);
    }
    friend constexpr difference_type operator-(const ReverseIterator& a, const ReverseIterator& b) {
      return b.iter_ - a.iter_;
    }
    friend constexpr bool operator==(const ReverseIterator& a, const ReverseIterator& b) {
      return a.iter_ == b.iter_;
    }
    friend constexpr bool operator!=(const ReverseIterator& a, const ReverseIterator& b) {
      return !(a == b);
    }

   private:
    /// Random-access iterator
    TIterator iter_;
  };

  FixedMap() : data_{nullptr}, capacity_{0}, size_{0} {}
  FixedMap(const FixedMap& other) = delete;
  FixedMap(FixedMap&& other) : data_{nullptr}, capacity_{0}, size_{0} { *this = std::move(other); }
  FixedMap& operator=(const FixedMap& other) = delete;
  FixedMap& operator=(FixedMap&& other) {
    if (this != &other) {
      std::swap(data_, other.data_);
      std::swap(capacity_, other.capacity_);
      std::swap(size_, other.size_);
    }
    return *this;
  }
  ~FixedMap() {
    clear();
    DeallocateArray<Bucket>(data_);
  }

  bool operator==(const FixedMap& other) const {
    if ((capacity_ != other.capacity_) || (size_ != other.size_)) {
      return false;
    }
    for (size_type i = 0; i < capacity_; i++) {
      if (data_[i].occupied) {
        if (!other.data_[i].occupied || (data_[i].value != other.data_[i].value)) {
          return false;
        }
      }
    }
    return true;
  }
  bool operator!=(const FixedMap& other) const { return !(*this == other); }

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

  Expected<mapped_type&> operator[](const key_type& key) {
    Bucket* bucket = findMatchingBucket(key);
    if (!bucket) {
      auto result = emplace(std::make_pair(key, mapped_type()));
      if (!result) {
        return Unexpected<Error>{result.error()};
      }
      bucket = findMatchingBucket(key);
      if (!bucket) {
        return Unexpected<Error>{Error::kInvalidKey};
      }
    }
    return bucket->value.second;
  }

  // Returns the number of elements the container can currently hold
  size_type capacity() const { return capacity_; }
  // Returns the number of elements in the container
  size_type size() const { return size_; }
  // Returns true if the container has no elements
  bool empty() const { return size_ == 0; }
  // Returns true if the container has reached capacity
  bool full() const { return size_ == capacity_; }
  // Returns the load factor of the container
  double load_factor() const {
    return capacity_ > 0 ? static_cast<double>(size_) / static_cast<double>(capacity_) : 0.0;
  }

  // Checks if the container has a value for the given key
  bool contains(const key_type& key) const { return findMatchingBucket(key) != nullptr; }

  // Returns a reference to the value at the given key
  Expected<mapped_type&> at(const key_type& key) {
    Bucket* bucket = findMatchingBucket(key);
    if (!bucket) {
      return Unexpected<Error>{Error::kInvalidKey};
    }
    return bucket->value.second;
  }

  // Returns a read-only reference to the value with the given key
  Expected<const mapped_type&> at(const key_type& key) const {
    Bucket* bucket = findMatchingBucket(key);
    if (!bucket) {
      return Unexpected<Error>{Error::kInvalidKey};
    }
    return bucket->value.second;
  }

  // Update a key that is already in the map
  Expected<void> update(const key_type& key, const mapped_type& value) {
    Bucket* bucket = findMatchingBucket(key);
    if (!bucket) { return Unexpected<Error>{Error::kInvalidKey}; }
    bucket->value.second = value;
    return kSuccess;
  }

  // Assings an the value to the given key if it exists, else creates a new object
  template <typename... Args>
  Expected<void> insert_or_assign(Args&&... args) {
    value_type value(std::forward<Args>(args)...);
    Bucket* bucket = findMatchingBucket(value.first);
    if (bucket) {
      bucket->value.second = value.second;
      return kSuccess;
    } else {
      return emplace(std::move(value));
    }
  }

  // Creates a new object with the provided arguments and adds it using the given key
  template <typename... Args>
  Expected<void> emplace(Args&&... args) {
    value_type value(std::forward<Args>(args)...);
    if (findMatchingBucket(value.first)) {
      return Unexpected<Error>{Error::kDuplicateKey};
    }
    Bucket* bucket = findEmptyBucket(value.first);
    if (!bucket) {
      return Unexpected<Error>{Error::kContainerFull};
    }
    InplaceMoveConstruct<value_type>(BytePointer(&bucket->value), std::move(value));
    bucket->occupied = true;
    size_++;
    return kSuccess;
  }

  // Copies the object with the given key
  Expected<void> insert(const value_type& value) { return emplace(value); }
  // Moves the object with the given key
  Expected<void> insert(value_type&& value) {
    return emplace(std::forward<value_type>(value));
  }

  // Removes the object at the given key and destroys it
  Expected<void> erase(const key_type& key) {
    Bucket* bucket = findMatchingBucket(key);
    if (!bucket) {
      return Unexpected<Error>{Error::kInvalidKey};
    }
    Destruct<value_type>(BytePointer(&bucket->value));
    bucket->occupied = false;
    size_--;
    return kSuccess;
  }

  // Removes all objects from the vector
  void clear() {
    for (size_type i = 0; i < capacity_; i++) {
      Bucket* bucket = &data_[i];
      if (bucket->occupied) {
        Destruct<value_type>(BytePointer(&bucket->value));
        bucket->occupied = false;
      }
    }
    size_ = 0;
  }

  // Allocates memory to hold the specified number of elements
  Expected<void> reserve(size_type capacity) {
    if (capacity > capacity_) {
      Bucket* const temp_data = data_;
      const size_type temp_capacity = capacity_;
      const size_type temp_size = size_;

      data_ = AllocateArray<Bucket>(capacity);
      capacity_ = capacity;
      size_ = 0;

      if (data_ == nullptr) {
        data_ = temp_data;
        capacity_ = temp_capacity;
        size_ = temp_size;
        return Unexpected<Error>{Error::kOutOfMemory};
      }

      for (size_type i = 0; i < capacity_; i++) {
        data_[i].occupied = false;
      }

      for (size_type i = 0; i < temp_capacity; i++) {
        Bucket* bucket = &temp_data[i];
        if (bucket->occupied) {
          auto result = emplace(std::move(bucket->value));
          if (!result) {
            return Unexpected<Error>{result.error()};
          }
        }
      }

      DeallocateArray<Bucket>(temp_data);
    }

    return kSuccess;
  }

  // Copies the contents of the given map
  // Current contents are discarded
  // Fails if given map is larger than current capacity
  Expected<void> copy_from(const FixedMap& other) {
    if (other.size_ > capacity_) {
      return Unexpected<Error>{Error::kContainerFull};
    }
    clear();
    for (size_type i = 0; i < other.capacity_; i++) {
      Bucket* bucket = &other.data_[i];
      if (bucket->occupied) {
        auto result = emplace(bucket->value.first, bucket->value.second);
        if (!result) {
          return Unexpected<Error>{result.error()};
        }
      }
    }
    return kSuccess;
  }

 private:
  // Container for a key-value pair
  struct Bucket {
    value_type value;
    bool occupied;
  };

  // Searches for a matching bucket with the given key
  Bucket* findMatchingBucket(const key_type& key) const {
    if (capacity_ == 0) {
      return nullptr;
    }

    size_type index = hash(key);
    const size_type start = index;
    Bucket* bucket = &data_[index];

    // Look for a matching bucket
    while (bucket->occupied) {
      if (bucket->value.first == key) {
        return bucket;
      }

      index = (index + 1) % capacity_;
      bucket = &data_[index];

      if (index == start) {
        return nullptr;
      }
    }

    return nullptr;
  }

  // Searches for an empty bucket with the given key
  Bucket* findEmptyBucket(const key_type& key) const {
    if (capacity_ == 0) {
      return nullptr;
    }

    size_type index = hash(key);
    const size_type start = index;
    Bucket* bucket = &data_[index];

    // Look for an empty bucket
    while (bucket->occupied) {
      index = (index + 1) % capacity_;
      bucket = &data_[index];

      if (index == start) {
        return nullptr;
      }
    }

    return bucket;
  }

  // Computes a hash of the given key that can be used as a table index
  size_type hash(const key_type& key) const { return Hash{}(key) % capacity_; }

  // Special value for returning a success
  const Expected<void> kSuccess{};

  // Pointer to an array of entries
  Bucket* data_;
  // Maximum number of entries the container can hold
  size_type capacity_;
  // Number of entries stored
  size_type size_;
};

}  // namespace nvidia

#endif  // NVIDIA_GXF_COMMON_FIXED_MAP_HPP_
