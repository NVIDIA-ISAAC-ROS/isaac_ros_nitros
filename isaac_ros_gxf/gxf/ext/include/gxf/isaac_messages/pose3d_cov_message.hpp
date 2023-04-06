/*
Copyright (c) 2023, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#ifndef NVIDIA_ISAAC_EXTENSIONS_MESSAGES_POSE3D_COV_MESSAGE_HPP_
#define NVIDIA_ISAAC_EXTENSIONS_MESSAGES_POSE3D_COV_MESSAGE_HPP_

#include "engine/core/math/pose3.hpp"
#include "engine/core/tensor/tensor.hpp"
#include "gems/common/pose_frame_uid.hpp"
#include "gxf/core/entity.hpp"
#include "gxf/core/expected.hpp"
#include "gxf/std/allocator.hpp"
#include "gxf/std/tensor.hpp"
#include "gxf/std/timestamp.hpp"

namespace nvidia {
namespace isaac {

// Stores the 3D Pose, its base frame UID, its covariance matrix and timestamp
struct Pose3dCovMessageParts {
  // The message entity
  gxf::Entity entity;
  // The pose, ie, the transform that moves a point from child to parent coordinate
  // p_parent = transform_parent_to_child * p_child
  gxf::Handle<::isaac::Pose3d> pose;
  // The pose covariance represented as 6-by-6 tensor.
  // Row-major representation of the 6x6 covariance matrix
  // The orientation parameters use a fixed-axis representation.
  // In order, the parameters are:
  // (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
  // Precision is double
  // The below object is a view to the covariance tensor
  ::isaac::CpuTensorView2d covariance;
  // The parent frame UID.
  // This the reference frame UID for the pose in this message.
  gxf::Handle<PoseFrameUid> parent_pose_frame_uid;
  // The child/this frame UID.
  gxf::Handle<PoseFrameUid> child_pose_frame_uid;
  // Timestamp for this message.
  gxf::Handle<gxf::Timestamp> timestamp;
};

// Allocates a message entity representing Pose3dCovMessageParts.
gxf::Expected<Pose3dCovMessageParts> CreatePose3dCovMessage(
    gxf_context_t context, gxf::Handle<gxf::Allocator> allocator);

// Parses a gxf entity into a Pose3dCovMessageParts.
gxf::Expected<Pose3dCovMessageParts> GetPose3dCovMessage(gxf::Entity message);

}  // namespace isaac
}  // namespace nvidia

#endif  // NVIDIA_ISAAC_EXTENSIONS_MESSAGES_POSE3D_COV_MESSAGE_HPP_
