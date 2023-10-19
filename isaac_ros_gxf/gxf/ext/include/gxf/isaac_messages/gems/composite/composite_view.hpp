// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2020-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#pragma once

#include <array>
#include <cstdint>
#include <type_traits>

#include "engine/core/math/types.hpp"

namespace nvidia {
namespace isaac {

// Forward declare for operator=
template <typename K, int32_t N>
struct CompositeContainerConstPointer;
template <typename K, int32_t N>
struct CompositeContainerPointer;
template <typename K, int32_t N>
struct CompositeContainerArray;
template <typename K, int32_t N>
struct CompositeContainerEigen;

// A base class for an immutable composite view which uses a const pointer to access data.
template <typename K, int32_t N>
struct CompositeContainerConstPointer {
  CompositeContainerConstPointer() : pointer(nullptr) {}
  CompositeContainerConstPointer(const K* p) : pointer(p) {}

  static_assert(std::is_arithmetic<K>::value, "Composite only supports arithmetic types");

  using scalar_t = K;
  using Scalar = K;
  static constexpr int32_t kDimension = N;

  constexpr int32_t size() const { return N; }

  K operator[](int32_t i) const { return pointer[i]; }

  const K* pointer;
};

// Accesses an element in a composite
template <int32_t I, typename K, int32_t N>
K get(const CompositeContainerConstPointer<K, N>& state) {
  static_assert(0 <= I && I < N, "Index out of bounds");
  return state.pointer[I];
}

// A base class for a mutable composite view which uses a pointer to access data.
template <typename K, int32_t N>
struct CompositeContainerPointer {
  CompositeContainerPointer() : pointer(nullptr) {}
  CompositeContainerPointer(K* p) : pointer(p) {}

  static_assert(std::is_arithmetic<K>::value, "Composite only supports arithmetic types");

  using scalar_t = K;
  using Scalar = K;
  static constexpr int32_t kDimension = N;

  constexpr int32_t size() const { return N; }

  K operator[](int32_t i) const { return pointer[i]; }
  K& operator[](int32_t i) { return pointer[i]; }

  CompositeContainerPointer<K, N>& operator=(const CompositeContainerConstPointer<K, N>& other) {
    for (int32_t i = 0; i < size(); ++i) {
      pointer[i] = other[i];
    }
    return *this;
  }
  CompositeContainerPointer<K, N>& operator=(const CompositeContainerArray<K, N>& other) {
    for (int32_t i = 0; i < size(); ++i) {
      pointer[i] = other[i];
    }
    return *this;
  }
  CompositeContainerPointer<K, N>& operator=(const CompositeContainerEigen<K, N>& other) {
    for (int32_t i = 0; i < size(); ++i) {
      pointer[i] = other[i];
    }
    return *this;
  }

  K* pointer;
};

// Accesses an element in a composite
template <int32_t I, typename K, int32_t N>
K get(const CompositeContainerPointer<K, N>& state) {
  static_assert(0 <= I && I < N, "Index out of bounds");
  return state.pointer[I];
}

// Accesses an element in a composite
template <int32_t I, typename K, int32_t N>
K& get(CompositeContainerPointer<K, N>& state) {
  static_assert(0 <= I && I < N, "Index out of bounds");
  return state.pointer[I];
}

// A base class for a composite which uses std::array to store data.
template <typename K, int32_t N>
struct CompositeContainerArray {
  static_assert(std::is_arithmetic<K>::value, "Composite only supports arithmetic types");

  using scalar_t = K;
  using Scalar = K;
  static constexpr int32_t kDimension = N;

  constexpr int32_t size() const { return N; }

  K operator[](int32_t i) const { return data[i]; }
  K& operator[](int32_t i) { return data[i]; }

  CompositeContainerArray<K, N>& operator=(const CompositeContainerConstPointer<K, N>& other) {
    for (int32_t i = 0; i < size(); ++i) {
      data[i] = other[i];
    }
    return *this;
  }
  CompositeContainerArray<K, N>& operator=(const CompositeContainerPointer<K, N>& other) {
    for (int32_t i = 0; i < size(); ++i) {
      data[i] = other[i];
    }
    return *this;
  }

  std::array<K, N> data;
};

// A base class for a composite which uses ::nvidia::isaac::Vector to store data.
template <typename K, int32_t N>
struct CompositeContainerEigen {
  static_assert(std::is_arithmetic<K>::value, "Composite only supports arithmetic types");

  using scalar_t = K;
  using Scalar = K;
  static constexpr int32_t kDimension = N;

  constexpr int32_t size() const { return N; }

  K operator[](int32_t i) const { return elements[i]; }
  K& operator[](int32_t i) { return elements[i]; }

  CompositeContainerEigen<K, N>& operator=(const CompositeContainerConstPointer<K, N>& other) {
    for (int32_t i = 0; i < size(); ++i) {
      elements[i] = other[i];
    }
    return *this;
  }
  CompositeContainerEigen<K, N>& operator=(const CompositeContainerPointer<K, N>& other) {
    for (int32_t i = 0; i < size(); ++i) {
      elements[i] = other[i];
    }
    return *this;
  }

  ::nvidia::isaac::Vector<K, N> elements;
};

// Accesses an element in a composite
template <int32_t I, typename K, int32_t N>
K get(const CompositeContainerArray<K, N>& state) {
  return std::get<I>(state.data);
}

// Accesses an element in a composite
template <int32_t I, typename K, int32_t N>
K& get(CompositeContainerArray<K, N>& state) {
  return std::get<I>(state.data);
}

// Accesses an element in a composite
template <int32_t I, typename K, int32_t N>
K get(const CompositeContainerEigen<K, N>& state) {
  return state.elements[I];
}

// Accesses an element in a composite
template <int32_t I, typename K, int32_t N>
K& get(CompositeContainerEigen<K, N>& state) {
  return state.elements[I];
}

// An immutable composite view
template <typename K, typename Indices, template <typename, typename> typename ImmutableInterface>
using CompositeConstView = ImmutableInterface<K, CompositeContainerConstPointer<K, Indices::kSize>>;

// A mutable composite view
template <typename K, typename Indices, template <typename, typename> typename MutableInterface>
using CompositeView = MutableInterface<K, CompositeContainerPointer<K, Indices::kSize>>;

// A composite which owns its data
template <typename K, typename Indices, template <typename, typename> typename MutableInterface>
using Composite = MutableInterface<K, CompositeContainerArray<K, Indices::kSize>>;

// A composite which owns its data as an Eigen Vector
template <typename K, typename Indices, template <typename, typename> typename MutableInterface>
using CompositeEigen = MutableInterface<K, CompositeContainerEigen<K, Indices::kSize>>;

}  // namespace isaac
}  // namespace nvidia
