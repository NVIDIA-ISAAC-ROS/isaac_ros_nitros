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
  gxf::Handle<::nvidia::isaac::Pose3d> pose;
  // List of keypoints of the detected fiducial, in image space
  gxf::Handle<gxf::Tensor> keypoints;
};

// Message structure to organize a list of fiducials
struct FiducialListMessageParts
{
  gxf::Entity entity;
  FixedVector<gxf::Handle<FiducialInfo>, kMaxComponents> info;
  FixedVector<gxf::Handle<::nvidia::isaac::Pose3d>, kMaxComponents> pose;
  FixedVector<gxf::Handle<gxf::Tensor>, kMaxComponents> keypoints;
  size_t count;
};

// Creates a fiducial message entity and returns a view to it
// Message entity is activated by default
gxf::Expected<FiducialMessageParts> CreateFiducialMessage(gxf_context_t context);

// Parses a fiducial message entity and returns a view to it
gxf::Expected<FiducialMessageParts> GetFiducialMessage(gxf::Entity message);

// Creates a fiducial list message entity and returns a view to it
// Message entity is activated by default
gxf::Expected<FiducialListMessageParts> CreateFiducialListMessage(
  gxf_context_t context,
  size_t count);

// Parses a fiducial list message entity and returns a view to it
gxf::Expected<FiducialListMessageParts> GetFiducialListMessage(gxf::Entity message);

}  // namespace isaac
}  // namespace nvidia
