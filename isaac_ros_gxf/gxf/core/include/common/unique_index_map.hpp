/*
Copyright (c) 2020-2022, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#ifndef NVIDIA_GXF_COMMON_UNIQUE_INDEX_MAP_HPP_
#define NVIDIA_GXF_COMMON_UNIQUE_INDEX_MAP_HPP_

#include "common/expected.hpp"

namespace nvidia {

// Provides a basic fixed capacity container for tracking objects and assigning a unique id to every
// object inserted. It provides constant time insert/delete/access for all elements with the
// generated uid. This container guarantees that uids are never reused, and will return an
// Unexpected Error if it runs out of capacity.
template <typename T>
class UniqueIndexMap {
 private:
  // number of bits to use for holding the version number
  static constexpr uint64_t kVersionSize = 32;
  // number of bits to use for holding the index into the underlying container
  static constexpr uint64_t kIndexSize = 32;
  // bitmask for Version number
  static constexpr uint64_t kVersionMask = ((1ULL << kVersionSize) - 1ULL) << kIndexSize;
  // bitmask for Index number
  static constexpr uint64_t kIndexMask = (1ULL << kIndexSize) - 1ULL;

  static_assert(kIndexSize + kVersionSize == 64, "Invalid UID bit length");
  static_assert((kVersionMask ^ kIndexMask) == 0xFFFFFFFFFFFFFFFF,
                "bitmasks must cover full size of uint64_t");

 public:
  // Custom Error codes for construction and access of UniqueIndexMap map.
  enum struct Error {
    kArgumentInvalid,  // Invalid construction/initialization parameter
    kOutOfMemory,      // Could not allocate memory during initialization
    kContainerFull,    // UniqueIndexMap is full
    kNotFound          // UID does not exits in UniqueIndexMap
  };

  // Expected type for UniqueIndexMap which uses class specific Error type.
  template <typename U>
  using Expected = Expected<U, Error>;

  UniqueIndexMap() : uids_{nullptr}, data_{nullptr}, capacity_{0}, size_{0}, next_{0} {}
  UniqueIndexMap(const UniqueIndexMap<T>&) = delete;
  UniqueIndexMap(UniqueIndexMap<T>&& other)
      : uids_{other.uids_},
        data_{other.data_},
        capacity_{other.capacity_},
        size_{other.size_},
        next_{other.next_} {
    other.reset_values();
  }

  UniqueIndexMap<T>& operator=(const UniqueIndexMap<T>& other) = delete;
  UniqueIndexMap<T>& operator=(UniqueIndexMap<T>&& other) {
    uids_ = other.uids_;
    data_ = other.data_;
    capacity_ = other.capacity_;
    size_ = other.size_;
    next_ = other.next_;

    other.reset_values();
    return *this;
  }

  ~UniqueIndexMap() {
    for (uint64_t i = 0; i < capacity_; ++i) {
      if (GetIndex(uids_[i]) == i) {
        data_[i].~T();
      }
    }

    delete[] uids_;
    ::operator delete(data_);
  }

  // Allocate memory up to the provided capacity and initialize values for implicit free list.
  // Returns Unexpected<Error> if max_capacity is too large.
  Expected<void> initialize(uint64_t max_capacity) {
    if (max_capacity > (1ULL << kIndexSize)) {
      return Unexpected<Error>{Error::kArgumentInvalid};
    }

    capacity_ = max_capacity;

    uids_ = new (std::nothrow) uint64_t[max_capacity];
    data_ = static_cast<T*>(::operator new(max_capacity * sizeof(T), std::nothrow));

    if (uids_ == nullptr || data_ == nullptr) {
      return Unexpected<Error>{Error::kOutOfMemory};
    }

    for (uint64_t i = 0; i < max_capacity; ++i) {
      uids_[i] = i + 1;
    }
    return {};
  }

  // Check if there exists an object with the provided UID. Returns true if the uid exists in the
  // allocated range and if the version numbers match.
  bool valid(uint64_t uid) const {
    const uint64_t idx = GetIndex(uid);
    return (idx < capacity_) && (uids_[idx] == uid);
  }

  // Insert a new object into the container and return the generated UID via move constructor.
  // Return Unexpected<Error> if the object cannot be inserted.
  Expected<uint64_t> insert(T&& object) { return emplace(static_cast<T&&>(object)); }

  // Insert a new object into the container and return the generated UID via copy constructor.
  // Return Unexpected<Error> if the object cannot be inserted.
  Expected<uint64_t> insert(const T& object) { return emplace(object); }

  // Construct a new object with the provided arguments directly in the container, and return the
  // generated UID via copy. Return Unexpected<Error> if the object cannot be inserted.
  template <typename... Args>
  Expected<uint64_t> emplace(Args&&... args) {
    if (size_ == capacity_) {
      return Unexpected<Error>{Error::kOutOfMemory};
    }

    if (GetVersion(next_) == (1ULL << kVersionSize) - 1ULL) {
      return Unexpected<Error>{Error::kContainerFull};
    }

    const uint64_t idx = GetIndex(next_);
    const uint64_t temp = next_;
    next_ = uids_[idx];
    uids_[idx] = temp + (1ULL << kIndexSize);
    new (&data_[idx]) T{static_cast<Args&&>(args)...};
    ++size_;
    return uids_[idx];
  }

  // Erase the object tracked by UID. Return Unexpected<Error> if UID is not valid.
  Expected<void> erase(uint64_t uid) {
    if (!valid(uid)) {
      return Unexpected<Error>{Error::kNotFound};
    }

    uint64_t idx = GetIndex(uid);
    uids_[idx] = next_;
    next_ = uid;
    --size_;
    data_[idx].~T();

    return {};
  }

  // Get a pointer to the object tracked by UID. Returns an error if UID is not valid.
  Expected<T*> try_get(uint64_t uid) {
    if (!valid(uid)) {
      return Unexpected<Error>{Error::kNotFound};
    }
    return &data_[GetIndex(uid)];
  }

  // Get a pointer to the object tracked by UID. Returns an error if UID is not valid.
  Expected<const T*> try_get(uint64_t uid) const {
    if (!valid(uid)) {
      return Unexpected<Error>{Error::kNotFound};
    }
    return &data_[GetIndex(uid)];
  }

  // Performs a linear search and returns the UID of the object.
  // Returns an error if object is not found
  Expected<uint64_t> find(const T& object) const {
    for (size_t i = 0; i < capacity_; i++) {
      if (valid(uids_[i]) && data_[i] == object) {
        return uids_[i];
      }
    }
    return Unexpected<Error>{Error::kNotFound};
  }

  // Maximum capacity of the unique index map.
  inline uint64_t capacity() const { return capacity_; }

  // Current number of elements in the container.
  inline uint64_t size() const { return size_; }

 private:
  static uint64_t GetVersion(uint64_t uid) { return uid >> kIndexSize; }
  static uint64_t GetIndex(uint64_t uid) { return uid & kIndexMask; }

  // Resets all pointers and storage parameters to zero, without calling delete, primarily to be
  // used with move operators.
  void reset_values() {
    uids_ = nullptr;
    data_ = nullptr;
    capacity_ = 0;
    size_ = 0;
    next_ = 0;
  }

  uint64_t* uids_;     // ids with implicit list of free elements
  T* data_;            // objects storage
  uint64_t capacity_;  // maximum number of concurrent objects
  uint64_t size_;      // current number of objects
  uint64_t next_;      // next "free" element in the container
};

}  // namespace nvidia

#endif  // NVIDIA_GXF_COMMON_UNIQUE_INDEX_MAP_HPP_
