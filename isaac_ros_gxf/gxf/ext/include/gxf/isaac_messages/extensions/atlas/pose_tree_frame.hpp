/*
Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#ifndef NVIDIA_ISAAC_EXTENSIONS_ATLAS_POSE_TREE_FRAME_HPP_
#define NVIDIA_ISAAC_EXTENSIONS_ATLAS_POSE_TREE_FRAME_HPP_

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

 private:
  // Sets up the frame in PoseTree with given parameters, if it has not already. This function can
  // be called by the initialize() of this component or by the initialize() of other PoseTreeFrame
  // components, but it executes once per initialization.
  gxf_result_t setup();

  gxf::Parameter<std::string> frame_name_;
  gxf::Parameter<int32_t> number_edges_;
  gxf::Parameter<gxf::Handle<PoseTreeFrame>> parent_frame_;
  gxf::Parameter<::isaac::Pose3d> initial_pose_;
  gxf::Parameter<bool> invert_pose_;
  gxf::Parameter<int32_t> maximum_edge_history_;
  gxf::Parameter<gxf::Handle<PoseTree>> pose_tree_;
  gxf::Parameter<gxf::Handle<StringProvider>> namespace_;

  // The value returned by setup() function. Cleared at deinitialize().
  std::optional<gxf_result_t> setup_result_ = std::nullopt;
  // Cached values
  PoseTree::frame_t frame_uid_;
  std::string frame_name_string_;
};

}  // namespace isaac
}  // namespace nvidia

#endif  // NVIDIA_ISAAC_EXTENSIONS_ATLAS_POSE_TREE_FRAME_HPP_
