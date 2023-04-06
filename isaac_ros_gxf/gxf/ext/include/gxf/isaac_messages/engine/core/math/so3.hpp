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

namespace isaac {

// Class for 3D rotation
template <typename K>
class SO3 {
 public:
  using Scalar = K;
  static constexpr int kDimension = 3;

  // Creates an empty un-initialized quaternion
  // Note: Use Identity to create an identity rotation.
  SO3() {}

  // Return identity rotation
  static SO3 Identity() { return SO3(Quaternion<K>::Identity()); }
  // Creates a rotation which rotates by around the given axes by the magnitude of the axis
  static SO3 FromScaledAxis(const Vector3<K>& axis_angle) {
    const K norm = axis_angle.norm();
    if (IsAlmostZero(norm)) {
      return SO3::Identity();
    } else {
      return SO3(Quaternion<K>(Eigen::AngleAxis<K>(norm, axis_angle / norm)));
    }
  }
  // Creates a rotation which rotates by an angle around a given (not necessarily normalized) axis
  static SO3 FromAxisAngle(const Vector3<K>& axis, K angle) {
    return SO3(Quaternion<K>(Eigen::AngleAxis<K>(angle, axis.normalized())));
  }
  static SO3 FromAngleAxis(K angle, const Vector3<K>& axis) {
    return SO3(Quaternion<K>(Eigen::AngleAxis<K>(angle, axis.normalized())));
  }
  // Creates rotation from a (not necessarily normalized) quaternion
  static SO3 FromQuaternion(const Quaternion<K>& quaternion) {
    return SO3(quaternion.normalized());
  }
  // Creates rotation from a normalized quaternion
  // Note: This will assert if the quaternion does not have unit length.
  static SO3 FromNormalizedQuaternion(const Quaternion<K>& quaternion) {
    ASSERT(IsAlmostEqualRelative(quaternion.squaredNorm(), K(1)),
           "Given cos/sin vector is not normalized: %lf", quaternion.norm());
    return SO3(quaternion);
  }
  // Creates a 3D rotation from a 2D rotation in the XY plane
  static SO3 FromSO2XY(const SO2<K>& rotation) {
    return FromAxisAngle({K(0), K(0), K(1)}, rotation.angle());
  }
  // Creates rotation from a 3x3 rotation matrix
  static SO3 FromMatrix(const Matrix<K, 3, 3>& matrix) {
    return SO3(Quaternion<K>(matrix));
  }

  // Returns the rotation axis
  Vector3<K> axis() const { return quaternion_.coeffs().head(3).normalized(); }
  // Returns the angle of rotation around the axis
  // Note: This calls a trigonometric function.
  K angle() const {
    return K(2) * std::atan2(quaternion_.coeffs().head(3).norm(), quaternion_.coeffs().w());
  }
  // Returns the quaternion representation of the rotation
  const Quaternion<K>& quaternion() const { return quaternion_; }
  // Returns the matrix representation of the rotation
  Matrix3<K> matrix() const { return quaternion_.toRotationMatrix(); }

  // Returns the roll, pitch, yaw euler angles of the rotation
  Vector3<K> eulerAnglesRPY() const {
    const Vector3d euler_angles = quaternion_.toRotationMatrix().eulerAngles(0, 1, 2);
    // Make sure the roll is in range [-Pi/2, Pi/2]
    if (std::abs(euler_angles[0]) > Pi<K> * K(0.5)) {
      return Vector3d(WrapPi<K>(euler_angles[0] + Pi<K>),
                      WrapPi<K>(Pi<K> - euler_angles[1]),
                      WrapPi<K>(euler_angles[2] + Pi<K>));
    }
    return euler_angles;
  }

  // Returns the inverse rotation.
  SO3 inverse() const {
    // conjugate == inverse iff quaternion_.norm() == 1
    return SO3(quaternion_.conjugate());
  }

  // Casts to a different type
  template <typename S, typename std::enable_if_t<!std::is_same<S, K>::value, int> = 0>
  SO3<S> cast() const {
    // We need to re-normalize in the new type
    return SO3<S>::FromQuaternion(quaternion().template cast<S>());
  }
  template <typename S, typename std::enable_if_t<std::is_same<S, K>::value, int> = 0>
  const SO3& cast() const {
    // Nothing to do as the type does not change
    return *this;
  }

  // Converts to a 2D rotation in the XY plane
  SO2<K> toSO2XY() const {
    // 2D rotation matrix:
    //   cos(a)   -sin(a)
    //   sin(a)    cos(a)
    // Quaternion to 3D rotation matrix:
    //   1 - 2*(qy^2 + qz^2)      2*(qx*qy - qz*qw)   ...
    //     2*(qx*qy + qz*qw)    1 - 2*(qx^2 + qz^2)   ...
    // It follows (modulo re-normalization):
    //   cos(a) = 1 - (qx^2 + qy^2 + 2*qz^2)
    //   sin(a) = 2*qz*qw
    // These formulas correspond to the half-angle formulas for sin/cos.
    const K qx = quaternion_.x();
    const K qy = quaternion_.y();
    const K qz = quaternion_.z();
    const K qw = quaternion_.w();
    const K cos_a = K(1) - (qx * qx + qy * qy + K(2) * qz * qz);
    const K sin_a = K(2) * qz * qw;
    return SO2<K>::FromDirection(cos_a, sin_a);
  }

  // Rotation composition
  friend SO3 operator*(const SO3& lhs, const SO3& rhs) {
    return FromQuaternion(lhs.quaternion_ * rhs.quaternion_);
  }
  // Rotates a vector 3D by the given rotation
  friend Vector3<K> operator*(const SO3& rot, const Vector3<K>& vec) {
    return (rot.quaternion_ * Quaternion<K>(K(0), vec.x(), vec.y(), vec.z()) *
            rot.quaternion_.conjugate())
        .coeffs()
        .head(3);
  }

  // Create rotation from roll/pitch/yaw Euler angles
  static SO3 FromEulerAnglesRPY(K roll_angle, K pitch_angle, K yaw_angle) {
    SO3 roll = SO3::FromAngleAxis(roll_angle, {K(1), K(0), K(0)});
    SO3 pitch = SO3::FromAngleAxis(pitch_angle, {K(0), K(1), K(0)});
    SO3 yaw = SO3::FromAngleAxis(yaw_angle, {K(0), K(0), K(1)});
    return roll * pitch * yaw;
  }
  // Create rotation from Euler angles in order (roll, pitch, yaw)
  static SO3 FromEulerAnglesRPY(const Vector3d& roll_pitch_yaw) {
    return FromEulerAnglesRPY(roll_pitch_yaw[0], roll_pitch_yaw[1], roll_pitch_yaw[2]);
  }

  // Computes the jacobian of the rotation of normal vector. Plane normal only has rotation
  // component.
  Matrix<K, 3, 4> vectorRotationJacobian(const Vector3<K>& n) const {
    // Ref: https://www.weizmann.ac.il/sci-tea/benari/sites/sci-tea.benari/files/uploads/
    // softwareAndLearningMaterials/quaternion-tutorial-2-0-1.pdf
    // Rotation Matrix in Quaternion (R):
    //     [w^2 + x^2 - y^2 - z^2    2xy - 2wz                 2wy + 2xz]
    //     [2wz + 2xy                w^2 - x^2 + y^2 - z^2     2yz - 2wx]
    //     [2xz - 2wy                2wx + 2yz                 w^2 - x^2 - y^2 + z^2]
    const K qx = quaternion_.x();
    const K qy = quaternion_.y();
    const K qz = quaternion_.z();
    const K qw = quaternion_.w();
    Matrix<K, 3, 4> result;
    result << K(2)  * (qw * n[0] - qz * n[1] + qy * n[2]),
              K(2)  * (qx * n[0] + qy * n[1] + qz * n[2]),
              K(-2) * (qy * n[0] - qx * n[1] - qw * n[2]),
              K(-2) * (qz * n[0] + qw * n[1] - qx * n[2]),

              K(2)  * (qz * n[0] + qw * n[1] - qx * n[2]),
              K(2)  * (qy * n[0] - qx * n[1] - qw * n[2]),
              K(2)  * (qx * n[0] + qy * n[1] + qz * n[2]),
              K(2)  * (qw * n[0] - qz * n[1] + qy * n[2]),

              K(-2) * (qy * n[0] - qx * n[1] - qw * n[2]),
              K(2)  * (qz * n[0] + qw * n[1] - qx * n[2]),
              K(-2) * (qw * n[0] - qz * n[1] + qy * n[2]),
              K(2)  * (qx * n[0] + qy * n[1] + qz * n[2]);
    return result;
  }

 private:
  SO3(const Quaternion<K>& quaternion) : quaternion_(quaternion) {}

  Quaternion<K> quaternion_;
};

using SO3d = SO3<double>;
using SO3f = SO3<float>;

}  // namespace isaac
