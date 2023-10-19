// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2018-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "engine/core/assert.hpp"
#include "engine/core/epsilon.hpp"
#include "engine/core/math/types.hpp"
#include "engine/core/math/utils.hpp"

namespace nvidia {
namespace isaac {

// Class for 2D rotation
template <typename K>
class SO2 {
 public:
  using Scalar = K;
  static constexpr int kDimension = 2;

  // Creates an undefined rotation
  // Note: Use Identity to create the identity rotation
  SO2() {}

  // Return identity rotation
  CUDA_BOTH static SO2 Identity() {
    SO2 q;
    q.cos_sin_ = Vector2<K>(K(1), K(0));
    return q;
  }
  // Creates rotation from an angle
  // Note: This uses calls to trigonometric functions.
  CUDA_BOTH static SO2 FromAngle(K angle) {
    SO2 q;
    q.cos_sin_ = Vector2<K>(std::cos(angle), std::sin(angle));
    return q;
  }
  // Creates rotation from a not necessarily normalized direction vector
  CUDA_BOTH static SO2 FromDirection(const Vector2<K>& direction) {
    SO2 q;
    const K norm = direction.norm();
    ASSERT(!IsAlmostZero(norm), "Direction vector must not be 0");
    q.cos_sin_ = direction / norm;
    return q;
  }
  CUDA_BOTH static SO2 FromDirection(K dx, K dy) {
    return FromDirection(Vector2<K>(dx, dy));
  }
  // Creates from a normalized cos/sin direction angle
  CUDA_BOTH static SO2 FromNormalized(const Vector2<K>& cos_sin) {
    ASSERT(IsAlmostEqualRelative(cos_sin.squaredNorm(), K(1)),
           "Given cos/sin vector is not normalized: %lf", cos_sin.norm());
    SO2 q;
    q.cos_sin_ = cos_sin;
    return q;
  }
  CUDA_BOTH static SO2 FromNormalized(K cos_angle, K sin_angle) {
    return FromNormalized(Vector2<K>(cos_angle, sin_angle));
  }

  // Access to cosine and sine of the rotation angle.
  // Note this is a simple getter and deos not call trigonometric functions.
  CUDA_BOTH K cos() const { return cos_sin_[0]; }
  CUDA_BOTH K sin() const { return cos_sin_[1]; }
  // Access to cos and sin of the rotation angle as a direction vector.
  const Vector2<K>& asDirection() const { return cos_sin_; }

  // Returns the rotation angle in range [-PI;PI]
  // Note: This uses a call to a trigonometric function.
  CUDA_BOTH K angle() const {
    return std::atan2(sin(), cos());
  }
  // Return Rotation matrix
  CUDA_BOTH Matrix2<K> matrix() const {
    Matrix2<K> m;
    m(0, 0) = cos();  m(0, 1) = -sin();
    m(1, 0) = sin();  m(1, 1) =  cos();
    return m;
  }
  // Inverse Rotation
  CUDA_BOTH SO2 inverse() const {
    return SO2::FromNormalized(cos(), -sin());
  }

  // Casts to a different type
  template<typename S, typename std::enable_if_t<!std::is_same<S, K>::value, int> = 0>
  CUDA_BOTH SO2<S> cast() const {
    // We need to re-normalize in the new type
    return SO2<S>::FromDirection(asDirection().template cast<S>());
  }
  template<typename S, typename std::enable_if_t<std::is_same<S, K>::value, int> = 0>
  CUDA_BOTH const SO2& cast() const {
    // Nothing to do as the type does not change
    return *this;
  }

  // Rotation composition
  CUDA_BOTH friend SO2 operator*(const SO2& lhs, const SO2& rhs) {
    return FromDirection(lhs.cos() * rhs.cos() - lhs.sin() * rhs.sin(),
                         lhs.sin() * rhs.cos() + lhs.cos() * rhs.sin());
  }
  // Rotates a vector 2D
  CUDA_BOTH friend Vector2<K> operator*(const SO2& lhs, const Vector2<K>& vec) {
    return Vector2<K>(lhs.cos() * vec[0] - lhs.sin() * vec[1],
                      lhs.sin() * vec[0] + lhs.cos() * vec[1]);
  }

 private:
  Vector2<K> cos_sin_;
};

using SO2d = SO2<double>;
using SO2f = SO2<float>;

}  // namespace isaac
}  // namespace nvidia
