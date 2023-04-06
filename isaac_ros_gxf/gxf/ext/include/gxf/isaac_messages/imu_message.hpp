/*
Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
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
