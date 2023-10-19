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

#include "engine/core/math/pose2.hpp"
#include "engine/core/math/so3.hpp"
#include "engine/core/math/types.hpp"

namespace nvidia {
namespace isaac {

// Class for 3D transformation.
template <typename K>
struct Pose3 {
  using Scalar = K;
  static constexpr int kDimension = 3;

  SO3<K> rotation;
  Vector3<K> translation;

  // Returns identity transformation
  CUDA_BOTH static Pose3 Identity() {
    return Pose3{SO3<K>::Identity(), Vector3<K>::Zero()};
  }
  // Creates a translation transformation
  CUDA_BOTH static Pose3 Translation(const Vector3<K>& translation) {
    return Pose3{SO3<K>::Identity(), translation};
  }
  CUDA_BOTH static Pose3 Translation(K x, K y, K z) {
    return Pose3{SO3<K>::Identity(), Vector3<K>{x, y, z}};
  }
  // Creates a translation transformation
  CUDA_BOTH static Pose3 Rotation(const Vector3<K>& axis, K angle) {
    return Pose3{SO3<K>::FromAxisAngle(axis, angle), Vector3<K>::Zero()};
  }
  // Creates a 3D pose from a 2D pose in the XY plane
  CUDA_BOTH static Pose3 FromPose2XY(const Pose2<K>& pose) {
    return Pose3{
      SO3<K>::FromSO2XY(pose.rotation),
      Vector3<K>{pose.translation.x(), pose.translation.y(), K(0)}
    };
  }
  // Creates a pose from matrix transformation
  CUDA_BOTH static Pose3 FromMatrix(const Matrix4<K>& matrix) {
    return Pose3{
      SO3<K>::FromNormalizedQuaternion(Quaternion<K>(matrix.template topLeftCorner<3, 3>())),
      matrix.template topRightCorner<3, 1>()};
  }

  // Returns the inverse transformation
  CUDA_BOTH Pose3 inverse() const {
    const auto inv = rotation.inverse();
    return Pose3{inv, -(inv * translation)};
  }

  // Returns the transformation as a matrix
  CUDA_BOTH Matrix4<K> matrix() const {
    Matrix4<K> ret;
    ret.template topLeftCorner<3, 3>() = rotation.matrix();
    ret.template topRightCorner<3, 1>() = translation;
    ret.template bottomLeftCorner<1, 3>().setZero();
    ret(3, 3) = K(1);
    return ret;
  }

  // Casts to a different type
  template<typename S, typename std::enable_if_t<!std::is_same<S, K>::value, int> = 0>
  CUDA_BOTH Pose3<S> cast() const {
    return Pose3<S>{rotation.template cast<S>(), translation.template cast<S>()};
  }
  template<typename S, typename std::enable_if_t<std::is_same<S, K>::value, int> = 0>
  CUDA_BOTH const Pose3& cast() const {
    // Nothing to do as the type does not change
    return *this;
  }

  // Converts to a 2D pose in the XY plane
  CUDA_BOTH Pose2<K> toPose2XY() const {
    return Pose2<K>{rotation.toSO2XY(), translation.template head<2>()};
  }

  // Composition of poses
  CUDA_BOTH friend Pose3 operator*(const Pose3& lhs, const Pose3& rhs) {
    return Pose3{lhs.rotation * rhs.rotation, lhs.rotation * rhs.translation + lhs.translation};
  }
  // Transforms a vector 2D with the given 2D transformation
  CUDA_BOTH friend Vector3<K> operator*(const Pose3& pose, const Vector3<K>& vec) {
    return pose.rotation * vec + pose.translation;
  }
  // Exponentaiton of the transformation
  CUDA_BOTH Pose3 pow(K exponent) const {
    // First step: align the rotation vector with the z axis.
    const K angle = rotation.angle();
    if (IsAlmostZero(angle)) {
      // TODO(bbutin): Use Taylor expension in that case?
      return Pose3{rotation, exponent * translation};
    }
    const Vector3<K> axis = rotation.axis();
    const Vector3<K> cross = axis.cross(Vector3<K>(K(0), K(0), K(1)));
    const K cos = axis.z();
    const K sin = cross.norm();
    Matrix3<K> rot = Matrix3<K>::Identity() * cos;
    // If axis is align in the Z direction, the matrix above will do the trick, if not we compute
    // rotation matrix to transform the axis in the Z axis.
    if (!IsAlmostZero(sin)) {
      // TODO(bbutin): Use Taylor expension in case sin ~ 0?
      const Vector3<K> unit = cross / sin;
      rot += (1.0 - cos) * unit * unit.transpose();
      rot(0, 1) += -cross.z();
      rot(0, 2) += cross.y();
      rot(1, 2) += -cross.x();
      rot(1, 0) += cross.z();
      rot(2, 0) += -cross.y();
      rot(2, 1) += cross.x();
    }
    // Now we can compute the exponentation the same way as for the 2d for x and y, while z will
    // just be linearly FromScaledAxis
    const K half_angle = angle / K(2);
    const K csc_sin = IsAlmostZero(angle)
        ? exponent * (K(1) + (K(1) - exponent * exponent) * half_angle * half_angle / K(6))
        : std::sin(half_angle * exponent) / std::sin(half_angle);
    const SO2<K> rot2 = SO2<K>::FromAngle(half_angle * (exponent - K(1)));
    const Vector3<K> t = rot * translation;
    const Vector2<K> xy = csc_sin * (rot2 * t.template head<2>());
    const SO3<K> final_rotation = SO3<K>::FromAngleAxis(angle * exponent, axis);
    return Pose3{final_rotation, rot.inverse() * Vector3<K>(xy.x(), xy.y(), t.z() * exponent)};
  }
};

using Pose3d = Pose3<double>;
using Pose3f = Pose3<float>;

// The "exponential" map of SE(3) which maps a tangent space element to the manifold space
// For SE(3) this function encodes the tangent space as a six-dimensional vector
// (px, py, pz, rx, ry, rz) where (px, py, pz) is the translation component and (rx, ry, rz) is the
// scaled rotation axis.
template <typename K>
CUDA_BOTH Pose3<K> Pose3Exp(const Vector6<K>& tangent) {
  return Pose3<K>{SO3<K>::FromScaledAxis(tangent.template tail<3>()), tangent.template head<3>()};
}

// The "logarithmic" map of SE(3) which maps a manifold space element to the tangent space
// This computes the tangent for a pose relative to the identity pose. Log and exp are inverse
// to each other.
template <typename K>
CUDA_BOTH Vector6<K> Pose3Log(const Pose3<K>& pose) {
  Vector6<K> result;
  result.template head<3>() = pose.translation;
  result.template tail<3>() = pose.rotation.angle() * pose.rotation.axis();
  return result;
}

// Returns true if given pose is almost identity.
template <typename K>
CUDA_BOTH bool IsPoseAlmostIdentity(const Pose3<K>& pose) {
  return IsAlmostZero(pose.translation.norm()) && IsAlmostZero(pose.rotation.angle());
}

// Returns distance to identity pose in position and angle.
// First element of the return value is for position, the seconds is for angle.
template <typename K>
CUDA_BOTH Vector2<K> PoseMagnitude(const Pose3<K>& pose) {
  return {pose.translation.norm(), static_cast<K>(std::abs(pose.rotation.angle()))};
}

}  // namespace isaac
}  // namespace nvidia
