/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>
#include <utility>

#include "engine/core/math/pose2.hpp"
#include "engine/core/math/pose3.hpp"
#include "engine/gems/geometry/pinhole.hpp"
#include "engine/gems/serialization/json.hpp"
#include "engine/gems/serialization/json_formatter.hpp"

namespace isaac {
namespace sight {

// Sight Operations Transformation:
// Contains the Pose2 or Pose3 transform as well as a scale factor
class SopTransform {
 public:
  // Default constructor
  SopTransform() = default;

  // Creates a SopTransform using the canvas pixel frame (allow you to draw directly at a given
  // position on a canvas).
  // Note: this only works for 2D and the Sop will not be rendered in 3D.
  static SopTransform CanvasFrame() {
    SopTransform t;
    t.json_["t"] = "c";
    return t;
  }

  // Creates a SopTransform using a reference frame from uid
  SopTransform(uint64_t uid) {
    json_["t"] = "f";
    json_["u"] = std::to_string(uid);
  }

  // Creates a SopTransform using a reference frame
  SopTransform(std::string reference_frame) {
    json_["t"] = "f";
    json_["f"] = std::move(reference_frame);
  }

  // Creates a SopTransform using a Pose2
  template <typename K>
  SopTransform(const Pose2<K>& pose) {
    json_["t"] = "2d";
    serialization::Set(json_["p"], pose);
  }

  // Creates a SopTransform using a Pose3
  template <typename K>
  SopTransform(const Pose3<K>& pose) {
    json_["t"] = "3d";
    serialization::Set(json_["p"], pose);
  }

  // Creates a transform with a Pose and a scale
  template <typename Pose>
  SopTransform(const Pose& pose, double scale) : SopTransform(pose) {
    json_["s"] = scale;
  }

  // Creates a transform with a Pose and a pinhole projection
  template <typename Pose, typename K>
  SopTransform(const Pose& pose, const geometry::Pinhole<K>& pinhole) : SopTransform(pose) {
    Json proj;
    proj["c0"] = pinhole.center[0];
    proj["c1"] = pinhole.center[1];
    proj["f0"] = pinhole.focal[0];
    proj["f1"] = pinhole.focal[1];
    json_["proj"] = proj;
  }

  // Creates a transform with a Pose and a scale
  template <typename Pose, typename K>
  SopTransform(const Pose& pose, double scale, const geometry::Pinhole<K>& pinhole)
  : SopTransform(pose, pinhole) {
    json_["s"] = scale;
  }

 private:
  friend const Json& ToJson(const SopTransform&);
  friend Json ToJson(SopTransform&&);

  Json json_;
};

// Returns the json of a SopTransform
const Json& ToJson(const SopTransform&);
Json ToJson(SopTransform&&);

}  // namespace sight
}  // namespace isaac
