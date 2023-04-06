/*
 * SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef NVIDIA_GXF_COMMON_SPAN_HPP_
#define NVIDIA_GXF_COMMON_SPAN_HPP_

#include "common/byte.hpp"
#include "common/expected.hpp"
#include "common/iterator.hpp"
#include "common/type_utils.hpp"

namespace nvidia {

namespace detail {

/// Type traits to enable comparing `Span<T>` with `Span<const T>`
template <class T, class U> using EnableIfComparable_t = EnableIf_t<
  IsSame_v<RemoveConst_t<T>, RemoveConst_t<U>>
>;

/// Type traits to determine if type stores elements in contiguous memory
template <class T> using EnableIfContiguous_t = EnableIf_t<Conjunction_v<HasData<T>, HasSize<T>>>;

}  // namespace detail

/// Container to hold a pointer and size for contiguous memory.
/// `Span` can be passed between functions and components and ensures that the size of the array is
/// always available and can be checked for safe memory access. `Span` only wraps existing memory.
template <typename T>
class Span {
 public:
  using element_type     = T;
  using value_type       = RemoveCV_t<element_type>;
  using size_type        = size_t;
  using iterator         = RandomAccessIterator<const Span, element_type>;
  using reverse_iterator = ReverseIterator<iterator>;

  /// Custom error codes for `Span`
  enum struct Error {
    kArgumentOutOfRange,
    kContainerEmpty,
  };

  /// `Expected` which uses `Span::Error`
  template <typename U>
  using Expected = Expected<U, Error>;

  constexpr Span(T* data, size_t size) : data_{data}, size_{data != nullptr ? size : 0} {}
  constexpr Span() : Span(nullptr, 0) {}
  template <size_t N>
  constexpr Span(T (&array)[N]) : Span(array, N) {}
  /// Conversion from other types that have `data()` and `size()` functions
  template <typename U, typename = detail::EnableIfContiguous_t<U>>
  constexpr Span(U& other) : Span(other.data(), other.size()) {}

  /// Implicit conversion from `Span<T>` to `Span<const T>`
  operator Span<const T>() const { return Span<const T>(data_, size_); }

  ~Span() = default;

  constexpr Span(const Span& other)      = default;
  constexpr Span(Span&& other)           = default;
  constexpr Span& operator=(const Span&) = default;
  constexpr Span& operator=(Span&&)      = default;

  constexpr iterator         begin()  const { return iterator(*this, 0); }
  constexpr iterator         end()    const { return iterator(*this, size_); }
  constexpr reverse_iterator rbegin() const { return reverse_iterator(end()); }
  constexpr reverse_iterator rend()   const { return reverse_iterator(begin()); }

  constexpr Expected<T&> operator[](size_t index) const { return at(index); }

  /// Returns a pointer to the underlying data
  constexpr T* data() const { return data_; }
  /// Returns the number of elements in the `Span`
  constexpr size_t size() const { return size_; }
  /// Returns the size of the `Span` in bytes
  constexpr size_t size_bytes() const { return size_ * sizeof(T); }
  /// Returns true if the `Span` has no elements
  constexpr bool empty() const { return size_ == 0; }

  /// Returns a reference to the element at `index`
  constexpr Expected<T&> at(size_t index) const {
    return index < size_ ? Expected<T&>{data_[index]}
                         : Unexpected<Error>{Error::kArgumentOutOfRange};
  }

  /// Returns a reference to the first element
  constexpr Expected<T&> front() const {
    return !empty() ? Expected<T&>{data_[0]}
                    : Unexpected<Error>{Error::kContainerEmpty};
  }

  /// Returns a reference to the last element
  constexpr Expected<T&> back() const {
    return !empty() ? Expected<T&>{data_[size_ - 1]}
                    : Unexpected<Error>{Error::kContainerEmpty};
  }

  /// Returns a `Span` consisting of `count` elements starting from `offset`
  constexpr Expected<Span> subspan(size_t offset, size_t count) const {
    return offset <= size_ && count <= size_ && offset + count <= size_
        ? Expected<Span>{Span(data_ + offset, count)}
        : Unexpected<Error>{Error::kArgumentOutOfRange};
  }

  /// Returns a `Span` starting from `offset`
  constexpr Expected<Span> subspan(size_t offset) const { return subspan(offset, size_ - offset); }
  /// Returns a `Span` consisting of the first `count` elements
  constexpr Expected<Span> first(size_t count) const { return subspan(0, count); }
  /// Returns a `Span` consisting of the last `count` elements
  constexpr Expected<Span> last(size_t count) const { return subspan(size_ - count, count); }

 private:
  /// Pointer to data
  T* data_;
  /// Size of data
  size_t size_;
};

template <typename T, typename U, typename = detail::EnableIfComparable_t<T, U>>
constexpr bool operator==(const Span<T>& a, const Span<U>& b) {
  if (a.size() != b.size()) {
    return false;
  }
  if (a.data() == b.data()) {
    return true;
  }
  for (size_t i = 0; i < a.size(); i++) {
    if (a.data()[i] != b.data()[i]) {
      return false;
    }
  }
  return true;
}

template <typename T, typename U, typename = detail::EnableIfComparable_t<T, U>>
constexpr bool operator!=(const Span<T>& a, const Span<U>& b) { return !(a == b); }

}  // namespace nvidia

#endif  // NVIDIA_GXF_COMMON_SPAN_HPP_
