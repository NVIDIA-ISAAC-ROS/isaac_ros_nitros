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

#include <string>

#include "engine/core/math/pose3.hpp"
#include "engine/core/optional.hpp"
#include "extensions/gxf_helpers/parameter_parser_isaac.hpp"
#include "extensions/gxf_helpers/parameter_wrapper_isaac.hpp"
#include "extensions/gxf_helpers/string_provider.hpp"
#include "gems/pose_tree/pose_tree.hpp"
#include "gxf/core/component.hpp"

namespace nvidia {
namespace isaac {

// Creates a frame in the PoseTree with given name. Optionally sets its pose. Provides methods to
// access its name and uid.
class PoseTreeFrame : public gxf::Component {
 public:
  gxf_result_t registerInterface(gxf::Registrar* registrar) override;
  gxf_result_t initialize() override;
  gxf_result_t deinitialize() override;

  // Returns UID of the frame
  PoseTree::frame_t frame_uid() const { return frame_uid_; }
  // Returns name of the frame
  const char* frame_name() const { return frame_name_string_.c_str(); }
  // Returns name of the parent frame
  const char* parent_frame_name() const;
  // Retursn whether the frame can be moved using interactive markers.
  bool interactive_marker() const { return interactive_marker_; }

 private:
  // Sets up the frame in PoseTree with given parameters, if it has not already. This function can
  // be called by the initialize() of this component or by the initialize() of other PoseTreeFrame
  // components, but it executes once per initialization.
  gxf_result_t setup();

  gxf::Parameter<std::string> frame_name_;
  gxf::Parameter<int32_t> number_edges_;
  gxf::Parameter<gxf::Handle<PoseTreeFrame>> parent_frame_;
  gxf::Parameter<::nvidia::isaac::Pose3d> initial_pose_;
  gxf::Parameter<bool> invert_pose_;
  gxf::Parameter<int32_t> maximum_edge_history_;
  gxf::Parameter<gxf::Handle<PoseTree>> pose_tree_;
  gxf::Parameter<gxf::Handle<StringProvider>> namespace_;
  gxf::Parameter<bool> interactive_marker_;

  // The value returned by setup() function. Cleared at deinitialize().
  std::optional<gxf_result_t> setup_result_ = std::nullopt;
  // Cached values
  PoseTree::frame_t frame_uid_;
  std::string frame_name_string_;
};

}  // namespace isaac
}  // namespace nvidia
