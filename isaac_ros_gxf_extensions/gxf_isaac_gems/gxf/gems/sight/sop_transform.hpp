// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <optional>
#include <string>
#include <utility>

#include "gems/core/math/pose2.hpp"
#include "gems/core/math/pose3.hpp"
#include "gems/geometry/pinhole.hpp"
#include "gems/sight/sop_serializer.hpp"

namespace nvidia {
namespace isaac {
namespace sight {

// Sight Operation Transform
// Contains the Pose2 or Pose3 transform or frame as well as a scale factor and a pinhole model.
struct SopTransform : public SopSerializer {
  // Creates empty transform
  SopTransform() = default;
  ~SopTransform() override = default;

  // Creates a SopTransform using the canvas pixel frame (allow you to draw directly at a given
  // position on a canvas).
  // Note: this only works for 2D and the Sop will not be rendered in 3D.
  static SopTransform CanvasFrame();

  // Creates a SopTransform using a reference frame from uid
  SopTransform(uint64_t uid);

  // Creates a SopTransform using a reference frame
  SopTransform(const std::string& reference_frame);

  // Creates a SopTransform using a Pose2
  template <typename K>
  SopTransform(const ::nvidia::isaac::Pose2<K>& pose) {
    pose2_ = pose.template cast<float>();
  }

  // Creates a SopTransform using a Pose3
  template <typename K>
  SopTransform(const ::nvidia::isaac::Pose3<K>& pose) {
    pose3_ = pose.template cast<float>();
  }

  // Creates a transform with a Pose and a scale
  template <typename Pose>
  SopTransform(const Pose& pose, double scale) : SopTransform(pose) {
    scale_ = scale;
  }

  // Creates a transform with a Pose and a pinhole projection
  template <typename Pose, typename K>
  SopTransform(const Pose& pose, const ::nvidia::isaac::geometry::Pinhole<K>& pinhole)
  : SopTransform(pose) {
    pinhole_ = pinhole.template cast<float>();
  }

  // Creates a transform with a Pose and a scale
  template <typename Pose, typename K>
  SopTransform(const Pose& pose, double scale, const ::nvidia::isaac::geometry::Pinhole<K>& pinhole)
  : SopTransform(pose, pinhole) {
    scale_ = scale;
  }

  // Helper == operator for testing deserialize.
  bool operator==(const SopTransform& style) const;

  bool toBinary(BufferSerialization& buffer) const override;

  bool fromBinary(BufferSerialization& buffer) override;

 private:
  bool canvas_ = false;
  std::optional<uint64_t> uid_;
  std::optional<std::string> frame_;
  std::optional<::nvidia::isaac::Pose2f> pose2_;
  std::optional<::nvidia::isaac::Pose3f> pose3_;
  std::optional<float> scale_;
  std::optional<::nvidia::isaac::geometry::Pinhole<float>> pinhole_;
};

}  // namespace sight
}  // namespace isaac
}  // namespace nvidia
