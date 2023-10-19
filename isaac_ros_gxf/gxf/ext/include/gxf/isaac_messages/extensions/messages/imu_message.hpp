// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "gems/common/pose_frame_uid.hpp"
#include "gxf/core/entity.hpp"
#include "gxf/core/expected.hpp"
#include "gxf/std/timestamp.hpp"

namespace nvidia {
namespace isaac {

// Contains the measurements of the IMU, i.e. linear acceleration and angular velocity.
struct Imu {
  // Linear accelerations of the IMU in its own frame.
  double linear_acceleration_x;
  double linear_acceleration_y;
  double linear_acceleration_z;

  // Angular velocities of the IMU in its own frame.
  double angular_velocity_x;
  double angular_velocity_y;
  double angular_velocity_z;
};

// Stores the measurements of the IMU provides a handle to its data.
struct ImuMessageParts {
  // The entity containing the whole message.
  gxf::Entity message;
  // Measurements of the IMU.
  gxf::Handle<Imu> imu;
  // Timestamp of publishing and acquisition.
  gxf::Handle<gxf::Timestamp> timestamp;
  // Reference pose frame of the measurements (linear acceleration and angular velocity).
  gxf::Handle<PoseFrameUid> pose_frame_uid;
};

gxf::Expected<ImuMessageParts> CreateImuMessage(gxf_context_t context);
gxf::Expected<ImuMessageParts> GetImuMessage(gxf::Entity message);

}  // namespace isaac
}  // namespace nvidia
