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
#include <utility>

#include "engine/core/math/types.hpp"

namespace isaac {
namespace geometry {

// Represent a Rectangular Cuboid in dimension N (https://en.wikipedia.org/wiki/Cuboid).
template <typename K, int N>
class NCuboid {
 public:
  // Type of a point.
  using Vector_t = Vector<K, N>;
  using Scalar = K;
  constexpr static int kDimension = N;

  // Creates a NCuboid from the boundaries:
  // A point will belong in the cuboid if for each dimension dims[i][0] <= pt[i] <= dims[i][1].
  static NCuboid FromBoundingCuboid(const std::array<Vector2<K>, N>& dims) {
    Vector_t min, max;
    for (size_t dim = 0; dim < N; dim++) {
      min[dim] = dims[dim][0];
      max[dim] = dims[dim][1];
    }
    return NCuboid{min, max};
  }

  // Creates a NCuboid from the boundaries:
  // A point will belong in the cuboid if for each dimension min[i] <= pt[i] <= max[i].
  static NCuboid FromOppositeCorners(const Vector_t& corner1, const Vector_t& corner2) {
    return NCuboid{corner1, corner2};
  }

  // Creates a NCuboid from the center and the size of each dimensions.
  // A point will belong in the cuboid if for each dimension:
  //    center[i] - sizes[i]/2 <= pt[i] <= center[i] + sizes[i]/2
  static NCuboid FromSizes(const Vector_t& center, const Vector_t& sizes) {
    const Vector_t min = center - sizes / K(2);
    return NCuboid{min, (min + sizes).eval()};
  }

  // Empty constructor to allow allocation
  NCuboid() {}

  // Accessor
  const Vector_t& min() const { return min_; }
  Vector_t& min() { return min_; }
  const Vector_t& max() const { return max_; }
  Vector_t& max() { return max_; }

  // Returns the center point of the cuboid
  Vector_t center() const { return (min_ + max_) / K(2); }

  // Returns sizes of each dimension of the cuboid
  Vector_t sizes() const { return max_ - min_; }

  // Returns whether a point is inside the cuboid
  bool isInside(const Vector_t& pt) const {
    for (size_t dim = 0; dim < kDimension; dim++) {
      if (pt[dim] < min_[dim] || pt[dim] > max_[dim]) {
        return false;
      }
    }
    return true;
  }

  // Casts to a different type
  template <typename S, typename std::enable_if_t<!std::is_same<S, K>::value, int> = 0>
  NCuboid<S, N> cast() const {
    return NCuboid<S, N>::FromOppositeCorners(min_.template cast<S>(), max_.template cast<S>());
  }
  template <typename S, typename std::enable_if_t<std::is_same<S, K>::value, int> = 0>
  const NCuboid& cast() const {
    // Nothing to do as the type does not change
    return *this;
  }

  // Computes Volume of cuboid. Computes the area if N is 2.
  Scalar volume() const {
    Scalar volume = Scalar(1);
    for (size_t dim = 0; dim < kDimension; dim++) {
      volume *= max_[dim] - min_[dim];
    }
    return volume;
  }

  // Extends this n_cuboid to contain the other n_cuboid
  void encapsulate(const NCuboid<K, N>& other) {
    min_ = min_.cwiseMin(other.min());
    max_ = max_.cwiseMax(other.max());
  }

  // Extends this n_cuboid to contain point
  void encapsulate(const Vector_t& point) {
    min_ = min_.cwiseMin(point);
    max_ = max_.cwiseMax(point);
  }

  // Translates the box
  void translate(const Vector_t& translation) {
    min_ += translation;
    max_ += translation;
  }

 private:
  // Private constructor to avoid confusion between corners initialization and sizes initialization.
  // Use FromOppositeCorners or FromSizes to create a line.
  NCuboid(const Vector_t& corner1, const Vector_t& corner2) {
    for (size_t dim = 0; dim < kDimension; dim++) {
      if (corner1[dim] < corner2[dim]) {
        min_[dim] = corner1[dim];
        max_[dim] = corner2[dim];
      } else {
        min_[dim] = corner2[dim];
        max_[dim] = corner1[dim];
      }
    }
  }
  Vector_t min_, max_;
};

template <typename K>
using Rectangle = NCuboid<K, 2>;
template <typename K>
using Box = NCuboid<K, 3>;

using RectangleD = Rectangle<double>;
using RectangleF = Rectangle<float>;
using RectangleI = Rectangle<int>;
using BoxD = Box<double>;
using BoxF = Box<float>;
using BoxI = Box<int>;

}  // namespace geometry
}  // namespace isaac
