/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <algorithm>
#include <cmath>
#include <type_traits>

#include "engine/core/math/types.hpp"
#include "engine/core/math/utils.hpp"

namespace isaac {
namespace geometry {

// Represent a line in dimension N.
// The line is defined with a point and a ray. If IsFullLine is set to false then the line is a
// half line starting from the origin and in the direction of the ray.
template <typename K, int N, bool IsFullLine>
class Line {
 public:
  // Type of a point.
  using Vector_t = Vector<K, N>;
  using Scalar = K;
  constexpr static int kDimension = N;

  // Empty constructor to allow allocation
  Line() {}

  // Helper to create a direction from a point and a direction.
  static Line FromDirection(const Vector_t& pt, const Vector_t& direction) {
    return Line{pt, direction};
  }
  // Helper to create a direction from two points.
  static Line FromPoints(const Vector_t& a, const Vector_t& b) {
    return Line{a, b-a};
  }

  // Returns a point on the line.
  const Vector_t& origin() const {
    return origin_;
  }

  // Returns the direction starting from the extremity 'origin'.
  const Vector_t& direction() const {
    return direction_;
  }

  // Clamp the value of lambda such as origin() + lambda * direction() belong on the Line.
  K clamp(K lambda) const {
    if (IsFullLine) {
      return lambda;
    } else {
      return std::max(K(0), lambda);
    }
  }

  // Casts to a different type
  template <typename S, typename std::enable_if_t<!std::is_same<S, K>::value, int> = 0>
  Line<S, N, IsFullLine> cast() const {
    return Line<S, N, IsFullLine>::FromDirection(
        origin_.template cast<S>(), direction_.template cast<S>());
  }
  template<typename S, typename std::enable_if_t<std::is_same<S, K>::value, int> = 0>
  const Line& cast() const {
    // Nothing to do as the type does not change
    return *this;
  }

 private:
  // Private constructor to avoid confusion between direction initialization and points
  // initialization. Use FromDirection or FromPoints to create a line.
  Line(const Vector_t& origin, const Vector_t& direction) : origin_(origin), direction_(direction) {
    ASSERT(!IsAlmostZero(direction.squaredNorm()), "direction must not be null");
  }

  Vector_t origin_;
  Vector_t direction_;
};

using Line2d = Line<double, 2, true>;
using Line3d = Line<double, 3, true>;
using Line4d = Line<double, 4, true>;
using Line2f = Line<float, 2, true>;
using Line3f = Line<float, 3, true>;
using Line4f = Line<float, 4, true>;

using Ray2d = Line<double, 2, false>;
using Ray3d = Line<double, 3, false>;
using Ray4d = Line<double, 4, false>;
using Ray2f = Line<float, 2, false>;
using Ray3f = Line<float, 3, false>;
using Ray4f = Line<float, 4, false>;

}  // namespace geometry
}  // namespace isaac
