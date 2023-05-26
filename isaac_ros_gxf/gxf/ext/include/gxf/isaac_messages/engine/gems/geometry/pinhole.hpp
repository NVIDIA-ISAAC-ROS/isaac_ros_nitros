/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <type_traits>

#include "engine/core/assert.hpp"
#include "engine/core/epsilon.hpp"
#include "engine/core/math/types.hpp"

namespace isaac {
namespace geometry {

// A pinhole camera model describes how points in 3D space are projected onto a 2D image plane.
//
// In Isaac we choose the following coordinate frames:
//   * Camera coordinate, i.e. a point in front of the pinhole which will be projected onto the
//     image plane, are given as:
//        X right, Y down, Z forward.
//   * Image coordinates, i.e. a point in pixel coordinates, are given as:
//        0: rows, 1: columns
//
// The pinhole type is templated on the scalar type used for optical center and focal length.
// This type must be a floating point type like float or double.
template <typename K, typename std::enable_if<std::is_floating_point<K>::value, int>::type = 0>
struct Pinhole {
  // The dimensions of the camera image plane in pixels in the order (rows, columns)
  Vector2i dimensions;
  // Focal length of the projection (in pixels) in the order (row, column)
  Vector2<K> focal;
  // Optical center of the projection (in pixels) in the order (row, column)
  Vector2<K> center;

  // Creates a pinhole camera model from horizontal field of view with center in the middle or the
  // image plane.
  static Pinhole FromHorizontalFieldOfView(const Vector2i& dimensions, K fov_horizontal) {
    ASSERT(fov_horizontal > K(0), "Field of view must be greater than 0");
    const K focal = static_cast<K>(dimensions[1]) / (K(2) * std::tan(K(0.5) * fov_horizontal));
    return Pinhole{dimensions, {focal, focal}, K(0.5) * dimensions.cast<K>()};
  }
  static Pinhole FromHorizontalFieldOfView(int rows, int cols, K fov_horizontal) {
    return FromHorizontalFieldOfView({rows, cols}, fov_horizontal);
  }
  // Similar to `FromHorizontalFieldOfView`, but the field of view angle indicates the vertical
  // field of view.
  static Pinhole FromVerticalFieldOfView(const Vector2i& dimensions, K fov_vertical) {
    ASSERT(fov_vertical > K(0), "Field of view must be greater than 0");
    const K focal = static_cast<K>(dimensions[0]) / (K(2) * std::tan(K(0.5) * fov_vertical));
    return Pinhole{dimensions, {focal, focal}, K(0.5) * dimensions.cast<K>()};
  }
  static Pinhole FromVerticalFieldOfView(int rows, int cols, K fov_vertical) {
    return FromVerticalFieldOfView({rows, cols}, fov_vertical);
  }

  // Projects a 3D point in camera coordinates onto the pixel plane and returns the pixel coordinate
  // as fractional values.
  template <typename Derived, typename std::enable_if<
                                  std::is_same<typename Derived::Scalar, K>::value, int>::type = 0>
  Vector2<K> project(const Eigen::MatrixBase<Derived>& point) const {
    ASSERT(!IsAlmostZero(point[2]), "Invaid point with z almost 0 (%f)", point[2]);
    return ((point.template head<2>() / point[2]).array()).reverse() * focal.array() +
           center.array();
  }
  // Projects a 3D point in camera coordinates onto the pixel plane and rouns the pixel coordinate
  // to the nearest pixel on the image plane (rounding down).
  template <typename Derived, typename std::enable_if<
                                  std::is_same<typename Derived::Scalar, K>::value, int>::type = 0>
  Vector2i projectToInt(const Eigen::MatrixBase<Derived>& point) const {
    const Vector2<K> pixel = project(point);
    return {FloorToInt(pixel[0]), FloorToInt(pixel[1])};
  }

  // Casts a ray through the given pixel coordinate and gives the point on the ray at the given
  // distance `depth`. The point is given in camera coordinates.
  template <typename Derived, typename std::enable_if<
                                  std::is_same<typename Derived::Scalar, K>::value, int>::type = 0>
  Vector3<K> unproject(const Eigen::MatrixBase<Derived>& pixel, K depth) const {
    const Vector2<K> a = ((pixel - center).array() / focal.array()) * depth;
    return {a[1], a[0], depth};
  }
  // Special version for integer-coordinate pixels. We assume that pixel rays go through the center
  // of the pixel.
  template <
      typename Derived,
      typename std::enable_if<std::is_same<typename Derived::Scalar, int>::value, int>::type = 0>
  Vector3<K> unproject(const Eigen::MatrixBase<Derived>& pixel, K depth) const {
    return unproject(pixel.template cast<K>() + Vector2<K>{K(0.5), K(0.5)}, depth);
  }
  Vector3<K> unproject(int px, int py, K depth) const {
    return unproject(Vector2i{px, py}.template cast<K>() + Vector2<K>{K(0.5), K(0.5)}, depth);
  }

  // Computes the area of a pixel at given depth assuming a planar patch
  K pixelAreaAtDepth(K depth) const { return depth * depth / (focal[0] * focal[1]); }

  // Scale the pinhole. The scale factor is different for rows and cols
  Pinhole<K> scale(const Vector2<K>& scale_factor) const {
    ASSERT((scale_factor.array() >= 0).all(), "Invalid scale factor");
    return { (dimensions.template cast<K>().cwiseProduct(scale_factor)).template cast<int>(),
             focal.cwiseProduct(scale_factor),
             center.cwiseProduct(scale_factor)};
  }

  // Scale the pinhole
  Pinhole<K> scale(K scale_factor) const {
    ASSERT(scale_factor > K(0), "Invalid scale factor");
    return scale(Vector2<K>{scale_factor, scale_factor});
  }

  // First crop the intrinsics and then scale them
  Pinhole<K> cropAndScale(const Vector2i& crop_start, const Vector2i& crop_size,
                          const Vector2i& scaled_size) const {
    ASSERT((crop_start.array() >= 0).all(), "Invalid crop start");
    ASSERT((crop_start.array() < dimensions.array()).all(), "Invalid crop start");
    ASSERT((crop_size.array() > 0).all(), "Invalid crop size");
    const Pinhole<K> cropped_pinhole{crop_size, focal, center - crop_start.cast<K>()};
    return cropped_pinhole.scale(scaled_size.cast<K>().cwiseQuotient(crop_size.cast<K>()));
  }

  // Computes the field of view angles (in radians) in the order (vertical, horizontal)
  Vector2<K> computeFieldOfView() const {
    return { K(2) * std::atan2(K(0.5) * static_cast<K>(dimensions[0] - 1), focal[0]),
             K(2) * std::atan2(K(0.5) * static_cast<K>(dimensions[1] - 1), focal[1]) };
  }

  // Returns the 3x3 projection matrix
  Matrix3<K> matrix() const {
    Matrix3<K> mat;
    mat << K(0), focal[0], center[0], focal[1], K(0), center[1], K(0), K(0), K(1);
    return mat;
  }

  // Casts the pinhole to a different numeric type. For example from double to float.
  template <typename S>
  Pinhole<S> cast() const {
    return {dimensions, focal.template cast<S>(), center.template cast<S>()};
  }
};

// Typedef for pinhole with using 64-bit floating points
using PinholeD = Pinhole<double>;

}  // namespace geometry
}  // namespace isaac
