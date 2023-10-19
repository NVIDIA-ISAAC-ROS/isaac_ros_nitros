// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2021-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <cstdint>

#include "engine/core/math/types.hpp"
#include "gems/composite/composite_view.hpp"
#include "gems/composite/schema.hpp"

namespace nvidia {
namespace isaac {

// Describes the full 2D pose and kinematic state of a Differential base
struct DifferentialBaseEgoMotionIndices {
  enum {
    // Cartesian position X (m)
    kPositionX,
    // Cartesian position Y (m)
    kPositionY,
    // In-plane rotation angle (radian)
    kHeading,
    // Linear (longitudinal) speed (m/s)
    kLinearSpeed,
    // Rotation speed (radian/s)
    kAngularSpeed,
    // Linear (longitudinal) acceleration (m/s^2)
    kLinearAcceleration,
    // Angular (rotaion) acceleration (radians/s^2)
    kAngularAcceleration,
    kSize
  };
};

inline composite::Schema DifferentialBaseEgoMotionCompositeSchema() {
  return composite::Schema({
    composite::Quantity::Vector("position", composite::Measure::kPosition, 2),
    composite::Quantity::Scalar("heading", composite::Measure::kRotation),
    composite::Quantity::Scalar("linear_speed", composite::Measure::kSpeed),
    composite::Quantity::Scalar("angular_speed", composite::Measure::kAngularSpeed),
    composite::Quantity::Scalar("linear_acceleration", composite::Measure::kAcceleration),
    composite::Quantity::Scalar("angular_acceleration", composite::Measure::kAngularAcceleration),
  });
}

template <typename K, typename Base>
struct DifferentialBaseEgoMotionImmutableInterface : public Base {
  using Base::operator=;
  using Vector2ConstMap = Eigen::Map<
      const ::nvidia::isaac::Vector2<K>, 0,
    Eigen::InnerStride<DifferentialBaseEgoMotionIndices::kPositionY
                       - DifferentialBaseEgoMotionIndices::kPositionX>>;
  K position_x() const { return get<DifferentialBaseEgoMotionIndices::kPositionX>(*this); }
  K position_y() const { return get<DifferentialBaseEgoMotionIndices::kPositionY>(*this); }
  K heading() const { return get<DifferentialBaseEgoMotionIndices::kHeading>(*this); }
  K linear_speed() const { return get<DifferentialBaseEgoMotionIndices::kLinearSpeed>(*this); }
  K angular_speed() const { return get<DifferentialBaseEgoMotionIndices::kAngularSpeed>(*this); }
  K linear_acceleration() const {
      return get<DifferentialBaseEgoMotionIndices::kLinearAcceleration>(*this); }
  K angular_acceleration() const {
      return get<DifferentialBaseEgoMotionIndices::kAngularAcceleration>(*this); }

  Vector2ConstMap position() const {
    return Vector2ConstMap(this->pointer + DifferentialBaseEgoMotionIndices::kPositionX);
  }
};

template <typename K, typename Base>
struct DifferentialBaseEgoMotionMutableInterface :
    public DifferentialBaseEgoMotionImmutableInterface<K, Base> {
  using DifferentialBaseEgoMotionImmutableInterface<K, Base>::operator=;
  using Vector2Map = Eigen::Map<::nvidia::isaac::Vector2<K>, 0,
                                Eigen::InnerStride<DifferentialBaseEgoMotionIndices::kPositionY
                                                   - DifferentialBaseEgoMotionIndices::kPositionX>>;
  K position_x() const { return get<DifferentialBaseEgoMotionIndices::kPositionX>(*this); }
  K position_y() const { return get<DifferentialBaseEgoMotionIndices::kPositionY>(*this); }
  K heading() const { return get<DifferentialBaseEgoMotionIndices::kHeading>(*this); }
  K angular_speed() const { return get<DifferentialBaseEgoMotionIndices::kAngularSpeed>(*this); }
  K linear_speed() const { return get<DifferentialBaseEgoMotionIndices::kLinearSpeed>(*this); }
  K angular_acceleration() const {
      return get<DifferentialBaseEgoMotionIndices::kAngularAcceleration>(*this); }
  K linear_acceleration() const {
      return get<DifferentialBaseEgoMotionIndices::kLinearAcceleration>(*this); }

  K& position_x() { return get<DifferentialBaseEgoMotionIndices::kPositionX>(*this); }
  K& position_y() { return get<DifferentialBaseEgoMotionIndices::kPositionY>(*this); }
  K& heading() { return get<DifferentialBaseEgoMotionIndices::kHeading>(*this); }
  K& angular_speed() { return get<DifferentialBaseEgoMotionIndices::kAngularSpeed>(*this); }
  K& linear_speed() { return get<DifferentialBaseEgoMotionIndices::kLinearSpeed>(*this); }
  K& angular_acceleration() {
      return get<DifferentialBaseEgoMotionIndices::kAngularAcceleration>(*this); }
  K& linear_acceleration() {
      return get<DifferentialBaseEgoMotionIndices::kLinearAcceleration>(*this); }

  Vector2Map position() { return Vector2Map(&position_x()); }
};

template <typename K>
using DifferentialBaseEgoMotionConstView = CompositeConstView<K, DifferentialBaseEgoMotionIndices,
                                                      DifferentialBaseEgoMotionImmutableInterface>;

template <typename K>
using DifferentialBaseEgoMotionView = CompositeView<K, DifferentialBaseEgoMotionIndices,
                                            DifferentialBaseEgoMotionMutableInterface>;

template <typename K>
using DifferentialBaseEgoMotion = Composite<K, DifferentialBaseEgoMotionIndices,
        DifferentialBaseEgoMotionMutableInterface>;

template <typename K>
using DifferentialBaseEgoMotionEigen =
  CompositeEigen<K, DifferentialBaseEgoMotionIndices, DifferentialBaseEgoMotionMutableInterface>;

// Describes the command type for a Differential base, ie. output of a controller and intput to the
// hardware driver
struct DifferentialBaseCommandIndices {
  enum {
    // Linear (longitudinal) speed (m/s)
    kLinearSpeed,
    // Rotation speed (radian/s)
    kAngularSpeed,
    kSize
  };
};

inline composite::Schema DifferentialBaseCommandCompositeSchema() {
  return composite::Schema({
    composite::Quantity::Scalar("linear_speed", composite::Measure::kSpeed),
    composite::Quantity::Scalar("angular_speed", composite::Measure::kAngularSpeed),
  });
}

template <typename K, typename Base>
struct DifferentialBaseCommandImmutableInterface : public Base {
  using Base::operator=;

  K linear_speed() const { return get<DifferentialBaseCommandIndices::kLinearSpeed>(*this); }
  K angular_speed() const { return get<DifferentialBaseCommandIndices::kAngularSpeed>(*this); }
};

template <typename K, typename Base>
struct DifferentialBaseCommandMutableInterface :
    public DifferentialBaseCommandImmutableInterface<K, Base> {
  using DifferentialBaseCommandImmutableInterface<K, Base>::operator=;

  K linear_speed() const { return get<DifferentialBaseCommandIndices::kLinearSpeed>(*this); }
  K angular_speed() const { return get<DifferentialBaseCommandIndices::kAngularSpeed>(*this); }

  K& linear_speed() { return get<DifferentialBaseCommandIndices::kLinearSpeed>(*this); }
  K& angular_speed() { return get<DifferentialBaseCommandIndices::kAngularSpeed>(*this); }
};

template <typename K>
using DifferentialBaseCommandConstView =
   CompositeConstView<K, DifferentialBaseCommandIndices, DifferentialBaseCommandImmutableInterface>;

template <typename K>
using DifferentialBaseCommandView =
    CompositeView<K, DifferentialBaseCommandIndices, DifferentialBaseCommandMutableInterface>;

template <typename K>
using DifferentialBaseCommand = Composite<K, DifferentialBaseCommandIndices,
                                      DifferentialBaseCommandMutableInterface>;

// Describes the kinematic state for an Differential base, for example input to base odometry
struct DifferentialBaseStateIndices {
  enum {
    // Linear (longitudinal) speed (m/s)
    kLinearSpeed,
    // Rotation speed (radian/s)
    kAngularSpeed,
    // Linear (longitudinal) acceleration (m/s^2)
    kLinearAcceleration,
    // Angular (rotational) acceleration (radians/s^2)
    kAngularAcceleration,
    kSize
  };
};

inline composite::Schema DifferentialBaseStateCompositeSchema() {
  return composite::Schema({
    composite::Quantity::Scalar("linear_speed", composite::Measure::kSpeed),
    composite::Quantity::Scalar("angular_speed", composite::Measure::kRotation),
    composite::Quantity::Scalar("linear_acceleration", composite::Measure::kAcceleration),
    composite::Quantity::Scalar("angular_acceleration", composite::Measure::kRotation)
  });
}

template <typename K, typename Base>
struct DifferentialBaseStateImmutableInterface : public Base {
  using Base::operator=;

  K linear_speed() const { return get<DifferentialBaseStateIndices::kLinearSpeed>(*this); }
  K angular_speed() const { return get<DifferentialBaseStateIndices::kAngularSpeed>(*this); }
  K linear_acceleration() const {
    return get<DifferentialBaseStateIndices::kLinearAcceleration>(*this); }
  K angular_acceleration() const {
    return get<DifferentialBaseStateIndices::kAngularAcceleration>(*this);
  }
};

template <typename K, typename Base>
struct DifferentialBaseStateMutableInterface
    : public DifferentialBaseStateImmutableInterface<K, Base> {
  using DifferentialBaseStateImmutableInterface<K, Base>::operator=;

  K linear_speed() const { return get<DifferentialBaseStateIndices::kLinearSpeed>(*this); }
  K angular_speed() const { return get<DifferentialBaseStateIndices::kAngularSpeed>(*this); }
  K linear_acceleration() const {
    return get<DifferentialBaseStateIndices::kLinearAcceleration>(*this); }
  K angular_acceleration() const {
    return get<DifferentialBaseStateIndices::kAngularAcceleration>(*this); }

  K& linear_speed() { return get<DifferentialBaseStateIndices::kLinearSpeed>(*this); }
  K& angular_speed() {
    return get<DifferentialBaseStateIndices::kAngularSpeed>(*this); }
  K& linear_acceleration() {
    return get<DifferentialBaseStateIndices::kLinearAcceleration>(*this); }
  K& angular_acceleration() {
    return get<DifferentialBaseStateIndices::kAngularAcceleration>(*this);
  }
};

template <typename K>
using DifferentialBaseStateConstView =
    CompositeConstView<K, DifferentialBaseStateIndices,
                       DifferentialBaseStateImmutableInterface>;

template <typename K>
using DifferentialBaseStateView =
    CompositeView<K, DifferentialBaseStateIndices, DifferentialBaseStateMutableInterface>;

template <typename K>
using DifferentialBaseState =
    Composite<K, DifferentialBaseStateIndices, DifferentialBaseStateMutableInterface>;

template <typename K>
using DifferentialBaseStateEigen =
    CompositeEigen<K, DifferentialBaseStateIndices, DifferentialBaseStateMutableInterface>;

}  // namespace isaac
}  // namespace nvidia
