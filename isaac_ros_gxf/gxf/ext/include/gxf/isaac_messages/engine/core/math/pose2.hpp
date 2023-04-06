/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/core/math/so2.hpp"
#include "engine/core/math/types.hpp"
#include "engine/core/math/utils.hpp"

namespace isaac {

// Class for 2D transformation.
template <typename K>
struct Pose2 {
  using Scalar = K;
  static constexpr int kDimension = 2;

  SO2<K> rotation;
  Vector2<K> translation;

  // Returns identity transformation
  static Pose2 Identity() {
    return Pose2{SO2<K>::Identity(), Vector2<K>::Zero()};
  }
  // Creates a translation transformation
  static Pose2 Translation(const Vector2<K>& translation) {
    return Pose2{SO2<K>::Identity(), translation};
  }
  static Pose2 Translation(K x, K y) {
    return Pose2{SO2<K>::Identity(), Vector2<K>{x, y}};
  }
  // Creates a rotation transformation
  static Pose2 Rotation(const K angle) {
    return Pose2{SO2<K>::FromAngle(angle), Vector2<K>::Zero()};
  }
  // Creates a pose from position and angle
  static Pose2 FromXYA(K px, K py, K angle) {
    return Pose2{SO2<K>::FromAngle(angle), Vector2<K>{px, py}};
  }

  // Returns the inverse transformation
  Pose2 inverse() const {
    const SO2<K> inv = rotation.inverse();
    return Pose2{inv, -(inv * translation)};
  }

  // Casts to a different type
  template<typename S, typename std::enable_if_t<!std::is_same<S, K>::value, int> = 0>
  Pose2<S> cast() const {
    return Pose2<S>{rotation.template cast<S>(), translation.template cast<S>()};
  }
  template<typename S, typename std::enable_if_t<std::is_same<S, K>::value, int> = 0>
  const Pose2& cast() const {
    // Nothing to do as the type does not change
    return *this;
  }

  // Composition of Pose2
  friend Pose2 operator*(const Pose2& lhs, const Pose2& rhs) {
    return Pose2{lhs.rotation * rhs.rotation, lhs.rotation * rhs.translation + lhs.translation};
  }
  // Transforms a vector 2D with the given 2D transformation
  friend Vector2<K> operator*(const Pose2& pose, const Vector2<K>& vec) {
    return pose.rotation * vec + pose.translation;
  }
};

using Pose2d = Pose2<double>;
using Pose2f = Pose2<float>;

// The "exponential" map from three-dimensional tangent space to SE(2) manifold space
// For SE(2) this function encodes the tangent space as a three-dimensional vector (tx, ty, a)
// where (tx, ty) is the translation component and a is the angle.
template <typename K>
Pose2<K> Pose2Exp(const Vector3<K>& tangent) {
  return Pose2<K>{SO2<K>::FromAngle(tangent[2]), tangent.template head<2>()};
}

// The "logarithmic" map from manifold to tangent space
// This computes the tangent for a pose relative to the identity pose. Log and exp are inverse
// to each other.
template <typename K>
Vector3<K> Pose2Log(const Pose2<K>& pose) {
  return Vector3<K>{pose.translation.x(), pose.translation.y(), pose.rotation.angle()};
}

// Returns true if given pose is almost identity.
template <typename K>
bool IsPoseAlmostIdentity(const Pose2<K>& pose) {
  return IsAlmostZero(pose.translation.x()) && IsAlmostZero(pose.translation.y()) &&
         IsAlmostOne(pose.rotation.cos());
}

// Returns distance to identity pose in position and angle.
// First element of the return value is for position, the seconds is for angle.
template <typename K>
Vector2<K> PoseMagnitude(const Pose2<K>& pose) {
  return {pose.translation.norm(), std::abs(pose.rotation.angle())};
}

}  // namespace isaac
