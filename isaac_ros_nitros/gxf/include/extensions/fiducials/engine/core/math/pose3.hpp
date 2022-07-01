/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/core/math/pose2.hpp"
#include "engine/core/math/so3.hpp"
#include "engine/core/math/types.hpp"

namespace isaac {

// Class for 3D transformation.
template <typename K>
struct Pose3 {
  using Scalar = K;
  static constexpr int kDimension = 3;

  SO3<K> rotation;
  Vector3<K> translation;

  // Returns identity transformation
  static Pose3 Identity() {
    return Pose3{SO3<K>::Identity(), Vector3<K>::Zero()};
  }
  // Creates a translation transformation
  static Pose3 Translation(const Vector3<K>& translation) {
    return Pose3{SO3<K>::Identity(), translation};
  }
  static Pose3 Translation(K x, K y, K z) {
    return Pose3{SO3<K>::Identity(), Vector3<K>{x, y, z}};
  }
  // Creates a translation transformation
  static Pose3 Rotation(const Vector3<K>& axis, K angle) {
    return Pose3{SO3<K>::FromAxisAngle(axis, angle), Vector3<K>::Zero()};
  }
  // Creates a 3D pose from a 2D pose in the XY plane
  static Pose3 FromPose2XY(const Pose2<K>& pose) {
    return Pose3{
      SO3<K>::FromSO2XY(pose.rotation),
      Vector3<K>{pose.translation.x(), pose.translation.y(), K(0)}
    };
  }

  // Returns the inverse transformation
  Pose3 inverse() const {
    const auto inv = rotation.inverse();
    return Pose3{inv, -(inv * translation)};
  }

  // Casts to a different type
  template<typename S, typename std::enable_if_t<!std::is_same<S, K>::value, int> = 0>
  Pose3<S> cast() const {
    return Pose3<S>{rotation.template cast<S>(), translation.template cast<S>()};
  }
  template<typename S, typename std::enable_if_t<std::is_same<S, K>::value, int> = 0>
  const Pose3& cast() const {
    // Nothing to do as the type does not change
    return *this;
  }

  // Converts to a 2D pose in the XY plane
  Pose2<K> toPose2XY() const {
    return Pose2<K>{rotation.toSO2XY(), translation.template head<2>()};
  }

  // Composition of poses
  friend Pose3 operator*(const Pose3& lhs, const Pose3& rhs) {
    return Pose3{lhs.rotation * rhs.rotation, lhs.rotation * rhs.translation + lhs.translation};
  }
  // Transforms a vector 2D with the given 2D transformation
  friend Vector3<K> operator*(const Pose3& pose, const Vector3<K>& vec) {
    return pose.rotation * vec + pose.translation;
  }
};

using Pose3d = Pose3<double>;
using Pose3f = Pose3<float>;

// The "exponential" map of SE(3) which maps a tangent space element to the manifold space
// For SE(3) this function encodes the tangent space as a six-dimensional vector
// (px, py, pz, rx, ry, rz) where (px, py, pz) is the translation component and (rx, ry, rz) is the
// scaled rotation axis.
template <typename K>
Pose3<K> Pose3Exp(const Vector6<K>& tangent) {
  return Pose3<K>{SO3<K>::FromScaledAxis(tangent.template tail<3>()), tangent.template head<3>()};
}

// The "logarithmic" map of SE(3) which maps a manifold space element to the tangent space
// This computes the tangent for a pose relative to the identity pose. Log and exp are inverse
// to each other.
template <typename K>
Vector6<K> Pose3Log(const Pose3<K>& pose) {
  Vector6<K> result;
  result.template head<3>() = pose.translation;
  result.template tail<3>() = pose.rotation.angle() * pose.rotation.axis();
  return result;
}

// Returns distance to identity pose in position and angle.
// First element of the return value is for position, the seconds is for angle.
template <typename K>
Vector2<K> PoseMagnitude(const Pose3<K>& pose) {
  return {pose.translation.norm(), static_cast<K>(std::abs(pose.rotation.angle()))};
}

}  // namespace isaac
