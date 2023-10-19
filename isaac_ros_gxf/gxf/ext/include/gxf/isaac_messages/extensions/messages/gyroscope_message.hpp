// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

// Contains the measurements of the gyroscope, i.e. angular velocity.
struct Gyroscope {
  // Angular velocities of the gyroscope in its own frame.
  double angular_velocity_x;
  double angular_velocity_y;
  double angular_velocity_z;
};

// Stores the measurements of the gyroscope provides a handle to its data.
struct GyroscopeMessageParts {
  // The entity containing the whole message.
  gxf::Entity message;
  // Measurements of the gyroscope.
  gxf::Handle<Gyroscope> gyroscope;
  // Timestamp of publishing and acquisition.
  gxf::Handle<gxf::Timestamp> timestamp;
  // Reference pose frame of the measurements (angular velocity).
  gxf::Handle<PoseFrameUid> pose_frame_uid;
};

gxf::Expected<GyroscopeMessageParts> CreateGyroscopeMessage(gxf_context_t context);
gxf::Expected<GyroscopeMessageParts> GetGyroscopeMessage(gxf::Entity message);

}  // namespace isaac
}  // namespace nvidia
