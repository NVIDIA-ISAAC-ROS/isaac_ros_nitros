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

namespace isaac {
namespace geometry {

// Represent a sphere in dimension N.
// Note: the mathematical definition of N-Sphere belongs in a space at N+1 dimensions. The N
// provided as template argument corresponds to the number of dimension.
// Therefore a circle is 1-sphere mathematically but NSphere<K, 2> here.
// Same for the regular sphere: 2-sphere mathematically but NSphere<K, 3> here.
template <typename K, int N>
struct NSphere {
 public:
  // Type of a point.
  using Vector_t = Vector<K, N>;
  using Scalar = K;
  constexpr static int kDimension = N;

  Vector_t center;
  Scalar radius;

  // Returns whether a point is inside the sphere
  bool isInside(const Vector_t& pt) const {
    return (center - pt).squaredNorm() <= radius * radius;
  }

  // Casts to a different type
  template <typename S, typename std::enable_if_t<!std::is_same<S, K>::value, int> = 0>
  NSphere<S, N> cast() const {
    return NSphere<S, N>{center.template cast<S>(), static_cast<S>(radius)};
  }
  template<typename S, typename std::enable_if_t<std::is_same<S, K>::value, int> = 0>
  const NSphere& cast() const {
    // Nothing to do as the type does not change
    return *this;
  }
};

template <typename K>
using Circle = NSphere<K, 2>;
template <typename K>
using Sphere = NSphere<K, 3>;

using CircleD = Circle<double>;
using CircleF = Circle<float>;
using SphereD = Sphere<double>;
using SphereF = Sphere<float>;

}  // namespace geometry
}  // namespace isaac
