/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <cmath>
#include <type_traits>

#include "engine/core/math/types.hpp"
#include "engine/core/math/utils.hpp"

namespace isaac {
namespace geometry {

// Represent a segment in dimension N.
// The segment is defined with a couple of points (both extremities).
template <typename K, int N>
class LineSegment {
 public:
  // Type of a point.
  using Vector_t = Vector<K, N>;
  using Scalar = K;
  constexpr static int kDimension = N;

  // Empty constructor to allow allocation.
  LineSegment() {}

  // Constructor.
  LineSegment(const Vector_t& a, const Vector_t& b) : a_(a), b_(b) {
  }

  // Helper to create a segment from two points.
  static LineSegment FromPoints(const Vector_t& a, const Vector_t& b) {
    return LineSegment{a, b};
  }

  // Returns one extremity such as origin() + direction() == the other extremity.
  const Vector_t& origin() const {
    return a_;
  }

  // Returns the direction starting from the extremity 'origin'.
  const Vector_t direction() const {
    return b_ - a_;
  }

  // Returns one extremity of the segment.
  const Vector_t& a() const {
    return a_;
  }

  // Returns one extremity of the segment.
  Vector_t& a() {
    return a_;
  }

  // Returns the other extremity of the segment.
  const Vector_t& b() const {
    return b_;
  }

  // Returns the other extremity of the segment.
  Vector_t& b() {
    return b_;
  }

  // Clamp the value of lambda such as origin() + lambda * direction() belong on the LineSegment.
  K clamp(K lambda) const {
    return lambda <= K(0) ? K(0) : (lambda < K(1) ? lambda : K(1));
  }

  // Casts to a different type
  template <typename S, typename std::enable_if_t<!std::is_same<S, K>::value, int> = 0>
  LineSegment<S, N> cast() const {
    return LineSegment<S, N>(a_.template cast<S>(), b_.template cast<S>());
  }
  template<typename S, typename std::enable_if_t<std::is_same<S, K>::value, int> = 0>
  const LineSegment& cast() const {
    // Nothing to do as the type does not change
    return *this;
  }

 private:
  Vector_t a_;
  Vector_t b_;
};

using LineSegment2i = LineSegment<int, 2>;
using LineSegment3i = LineSegment<int, 3>;
using LineSegment4i = LineSegment<int, 4>;
using LineSegment2d = LineSegment<double, 2>;
using LineSegment3d = LineSegment<double, 3>;
using LineSegment4d = LineSegment<double, 4>;
using LineSegment2f = LineSegment<float, 2>;
using LineSegment3f = LineSegment<float, 3>;
using LineSegment4f = LineSegment<float, 4>;

}  // namespace geometry
}  // namespace isaac
