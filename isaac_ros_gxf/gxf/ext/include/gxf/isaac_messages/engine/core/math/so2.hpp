/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/core/assert.hpp"
#include "engine/core/epsilon.hpp"
#include "engine/core/math/types.hpp"
#include "engine/core/math/utils.hpp"

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
  static SO2 Identity() {
    SO2 q;
    q.cos_sin_ = Vector2<K>(K(1), K(0));
    return q;
  }
  // Creates rotation from an angle
  // Note: This uses calls to trigonometric functions.
  static SO2 FromAngle(K angle) {
    SO2 q;
    q.cos_sin_ = Vector2<K>(std::cos(angle), std::sin(angle));
    return q;
  }
  // Creates rotation from a not necessarily normalized direction vector
  static SO2 FromDirection(const Vector2<K>& direction) {
    SO2 q;
    const K norm = direction.norm();
    ASSERT(!IsAlmostZero(norm), "Direction vector must not be 0");
    q.cos_sin_ = direction / norm;
    return q;
  }
  static SO2 FromDirection(K dx, K dy) {
    return FromDirection(Vector2<K>(dx, dy));
  }
  // Creates from a normalized cos/sin direction angle
  static SO2 FromNormalized(const Vector2<K>& cos_sin) {
    ASSERT(IsAlmostEqualRelative(cos_sin.squaredNorm(), K(1)),
           "Given cos/sin vector is not normalized: %lf", cos_sin.norm());
    SO2 q;
    q.cos_sin_ = cos_sin;
    return q;
  }
  static SO2 FromNormalized(K cos_angle, K sin_angle) {
    return FromNormalized(Vector2<K>(cos_angle, sin_angle));
  }

  // Access to cosine and sine of the rotation angle.
  // Note this is a simple getter and deos not call trigonometric functions.
  K cos() const { return cos_sin_[0]; }
  K sin() const { return cos_sin_[1]; }
  // Access to cos and sin of the rotation angle as a direction vector.
  const Vector2<K>& asDirection() const { return cos_sin_; }

  // Returns the rotation angle in range [-PI;PI]
  // Note: This uses a call to a trigonometric function.
  K angle() const {
    return std::atan2(sin(), cos());
  }
  // Return Rotation matrix
  Matrix2<K> matrix() const {
    Matrix2<K> m;
    m(0, 0) = cos();  m(0, 1) = -sin();
    m(1, 0) = sin();  m(1, 1) =  cos();
    return m;
  }
  // Inverse Rotation
  SO2 inverse() const {
    return SO2::FromNormalized(cos(), -sin());
  }

  // Casts to a different type
  template<typename S, typename std::enable_if_t<!std::is_same<S, K>::value, int> = 0>
  SO2<S> cast() const {
    // We need to re-normalize in the new type
    return SO2<S>::FromDirection(asDirection().template cast<S>());
  }
  template<typename S, typename std::enable_if_t<std::is_same<S, K>::value, int> = 0>
  const SO2& cast() const {
    // Nothing to do as the type does not change
    return *this;
  }

  // Rotation composition
  friend SO2 operator*(const SO2& lhs, const SO2& rhs) {
    return FromDirection(lhs.cos() * rhs.cos() - lhs.sin() * rhs.sin(),
                         lhs.sin() * rhs.cos() + lhs.cos() * rhs.sin());
  }
  // Rotates a vector 2D
  friend Vector2<K> operator*(const SO2& lhs, const Vector2<K>& vec) {
    return Vector2<K>(lhs.cos() * vec[0] - lhs.sin() * vec[1],
                      lhs.sin() * vec[0] + lhs.cos() * vec[1]);
  }

 private:
  Vector2<K> cos_sin_;
};

using SO2d = SO2<double>;
using SO2f = SO2<float>;

}  // namespace isaac
