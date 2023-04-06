/*
Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/core/math/pose3.hpp"
#include "extensions/fiducials/gems/fiducial_info.hpp"
#include "gxf/core/entity.hpp"
#include "gxf/core/expected.hpp"
#include "gxf/std/tensor.hpp"

namespace nvidia
{
namespace isaac
{

// A fiducial is an object placed in the field of view
// of an imaging system for use as a point of reference or a measure
struct FiducialMessageParts
{
  // The message entity
  gxf::Entity entity;
  // View to the FiducialsInfo instance holding meta information about this message
  gxf::Handle<FiducialInfo> info;
  // 3D pose of the detected tag from the camera coordinates,
  // consisting of orientation (quaternion) and translation
  // Camera coordinate (X - right, Y - down, Z - outward)
  // Tag coordinate (X - right, Y - down, Z - opposite to tag face direction)
  // Tag coordinates are centered at tag's upper-left corner
  // ie. Pose has identity quaternion and zero translation, when tag is facing the camera and it's
  // upper-left corner is centered at the camera center
  gxf::Handle<::isaac::Pose3d> pose;
  // List of keypoints of the detected fiducial, in image space
  gxf::Handle<gxf::Tensor> keypoints;
};

// Message structure to organize a list of fiducials
struct FiducialListMessageParts
{
  gxf::Entity entity;
  FixedVector<gxf::Handle<FiducialInfo>, kMaxComponents> info;
  FixedVector<gxf::Handle<::isaac::Pose3d>, kMaxComponents> pose;
  FixedVector<gxf::Handle<gxf::Tensor>, kMaxComponents> keypoints;
  size_t count;
};

// Creates a fiducial message entity and returns a view to it
// Message entity is activated by default
gxf::Expected<FiducialMessageParts> CreateFiducialMessage(
  gxf_context_t context,
  bool activate = true);

// Parses a fiducial message entity and returns a view to it
gxf::Expected<FiducialMessageParts> GetFiducialMessage(gxf::Entity message);

// Creates a fiducial list message entity and returns a view to it
// Message entity is activated by default
gxf::Expected<FiducialListMessageParts> CreateFiducialListMessage(
  gxf_context_t context,
  size_t count,
  bool activate = true);

// Parses a fiducial list message entity and returns a view to it
gxf::Expected<FiducialListMessageParts> GetFiducialListMessage(gxf::Entity message);

}  // namespace isaac
}  // namespace nvidia
