/*
 * SPDX-FileCopyrightText: Copyright (c) 2021-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef NVIDIA_GXF_MULTIMEDIA_CAMERA_HPP_
#define NVIDIA_GXF_MULTIMEDIA_CAMERA_HPP_

#include <array>
#include <cstdint>

namespace nvidia {
namespace gxf {

template<class T>
struct Vector2 {
    T x; /**< point x coordinate. */
    T y; /**< point y coordinate. */
};

using Vector2u = Vector2<uint32_t>;
using Vector2f = Vector2<float>;
using Vector2d = Vector2<double>;

enum class DistortionType {
  /**
   * No distortion
   */
  Perspective,
  /**
   * Brown model with 3 radial and 2 tangential distortion coefficients.
   */
  Brown,
  /**
   * Polynomial model with 6 radial and 2 tangential distortion coefficients.
   */
  Polynomial,
  /**
   * Fisheye model with 4 radial distortion coefficients.
   * Specifies the equidistant fisheye mapping.
   * Mapping is defined by:
   * \f[r = f\theta\f]
   * where:
   * - \f$\theta\f$ is the angle from the optical axis.
   * - \f$f\f$ is the focal length.
   * - \f$r\f$ is the distance of a pixel from the image center.
   */
  FisheyeEquidistant,
  /**
   * Fisheye model with 4 radial distortion coefficients.
   * Specifies the equisolid fisheye mapping.
   * Mapping is defined by:
   * \f[r = 2f\sin\left(\frac{\theta}{2}\right)\f]
   * where:
   * - \f$\theta\f$ is the angle from the optical axis.
   * - \f$f\f$ is the focal length.
   * - \f$r\f$ is the distance of a pixel from the image center.
   */
  FisheyeEquisolid,
  /**
   * Fisheye model with 4 radial distortion coefficients.
   * Specifies the orthographic fisheye mapping.
   * Mapping is defined by:
   * \f[r = f\sin(\theta)\f]
   * where:
   * - \f$\theta\f$ is the angle from the optical axis.
   * - \f$f\f$ is the focal length.
   * - \f$r\f$ is the distance of a pixel from the image center.
   */
  FisheyeOrthoGraphic,
  /**
   * Fisheye model with 4 radial distortion coefficients.
   * Specifies the stereographic fisheye mapping.
   * Mapping is defined by:
   * \f[r = 2f\tan\left(\frac{\theta}{2}\right)\f]
   * where:
   * - \f$\theta\f$ is the angle from the optical axis.
   * - \f$f\f$ is the focal length.
   * - \f$r\f$ is the distance of a pixel from the image center.
   */
  FisheyeStereographic,
};

/**
 * Camera intrinsics model.
 * Intrinsic camera matrix is defined as follow:
 *     [fx  s cx]
 * K = [ 0 fy cy]
 *     [ 0  0  1]
 * where:
 * - fx, fy represent focal length.
 * - cx, cy represent principle point.
 * - s represents the skew value.
 */
template <typename T>
struct CameraModelBase {
  // The maximum number of distortion coefficients.
  static constexpr int kMaxDistortionCoefficients = 8;
  // The dimensions of the camera image plane in pixels in the order x, y.
  Vector2u dimensions;
  // Focal length of the projection (in pixels) in the order x, y.
  Vector2<T> focal_length;
  // Optical center of the projection (in pixels) in the order x, y.
  Vector2<T> principal_point;
  // The skew coefficient between the x and the y axis (usually 0).
  T skew_value;
  // Distortion type of the camera.
  DistortionType distortion_type;
  // Distortion coefficients of the camera.
  T distortion_coefficients[kMaxDistortionCoefficients];
};

using CameraModel = CameraModelBase<float>;

/**
 * Camera extrinsics model.
 * Extrinsic camera matrix is defined as follow:
 * T = [R | t]
 * where:
 * - R is the rotation matrix.
 * - t is the translation vector.
 */
template <typename T>
struct Pose3DBase {
  // The 3x3 rotation matrix.
  std::array<T, 9> rotation;
  // The translation vector.
  std::array<T, 3> translation;
};

using Pose3D = Pose3DBase<float>;

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_MULTIMEDIA_CAMERA_HPP_
