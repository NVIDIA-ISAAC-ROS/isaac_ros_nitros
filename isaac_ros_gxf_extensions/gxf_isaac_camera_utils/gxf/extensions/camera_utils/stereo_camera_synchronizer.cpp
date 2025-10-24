// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "stereo_camera_synchronizer.hpp"

#include <limits>

#include "gems/gxf_helpers/expected_macro_gxf.hpp"
#include "gxf/std/timestamp.hpp"

namespace nvidia {
namespace isaac {

gxf_result_t StereoCameraSynchronizer::registerInterface(gxf::Registrar* registrar) {
  gxf::Expected<void> result;
  result &= registrar->parameter(
      rx_right_camera_, "rx_right_camera", "RX Right Camera Message",
      "Input for Right CameraMessage entities");
  result &= registrar->parameter(
      rx_left_camera_, "rx_left_camera", "RX Left Camera Message",
      "Input for Left CameraMessage entities");
  result &= registrar->parameter(
      tx_right_camera_, "tx_right_camera", "TX Right Camera Message",
      "Input for Right CameraMessage entities");
  result &= registrar->parameter(
      tx_left_camera_, "tx_left_camera", "TX Left Camera Message",
      "Input for Left CameraMessage entities");
  result &= registrar->parameter(
      max_timestamp_diff_, "max_timestamp_diff", "Max difference timestamps nanoseconds",
      "Max difference between timestamps in nanoseconds",
      static_cast<int64_t>(200000));  // 200 micro seconds
  return gxf::ToResultCode(result);
}

gxf_result_t StereoCameraSynchronizer::start() {
  // Set all previous timestamps to minimum value of int64_t
  // This ensures that the first message received does NOT trigger the
  // "not monotonically increasing" error
  prev_left_unnamed_timestamp_ = std::numeric_limits<int64_t>::min();
  prev_right_unnamed_timestamp_ = std::numeric_limits<int64_t>::min();
  prev_left_named_timestamp_ = std::numeric_limits<int64_t>::min();
  prev_right_named_timestamp_ = std::numeric_limits<int64_t>::min();
  return GXF_SUCCESS;
}

gxf_result_t StereoCameraSynchronizer::tick() {
  auto left_camera_entity = UNWRAP_OR_RETURN(rx_left_camera_->receive());
  auto right_camera_entity = UNWRAP_OR_RETURN(rx_right_camera_->receive());
  auto left_unnamed_timestamp = UNWRAP_OR_RETURN(left_camera_entity.get<gxf::Timestamp>());
  auto right_unnamed_timestamp = UNWRAP_OR_RETURN(right_camera_entity.get<gxf::Timestamp>());
  auto left_named_timestamp = UNWRAP_OR_RETURN(left_camera_entity.get<gxf::Timestamp>("timestamp"));
  auto right_named_timestamp = UNWRAP_OR_RETURN(
    right_camera_entity.get<gxf::Timestamp>("timestamp"));

  const int64_t unnamed_diff = std::abs(left_unnamed_timestamp->acqtime -
    right_unnamed_timestamp->acqtime);
  if (unnamed_diff > max_timestamp_diff_) {
    GXF_LOG_ERROR("L/R frames are not synchronized, unnamed timestamp diff = %ld ns", unnamed_diff);
    return GXF_FAILURE;
  }

  const int64_t named_diff = std::abs(left_named_timestamp->acqtime -
    right_named_timestamp->acqtime);
  if (named_diff > max_timestamp_diff_) {
    GXF_LOG_ERROR("L/R frames are not synchronized, named timestamp diff = %ld ns", named_diff);
    return GXF_FAILURE;
  }
  // Set right input timestamp to left input timestamp
  right_unnamed_timestamp->acqtime = left_unnamed_timestamp->acqtime;
  right_named_timestamp->acqtime = left_named_timestamp->acqtime;


  // Check if previous timestamp is lower than current timestamp
  if (prev_left_unnamed_timestamp_ > left_unnamed_timestamp->acqtime) {
    GXF_LOG_ERROR("Left unnamed timestamp is not monotonically increasing");
    return GXF_FAILURE;
  }
  if (prev_right_unnamed_timestamp_ > right_unnamed_timestamp->acqtime) {
    GXF_LOG_ERROR("Right unnamed timestamp is not monotonically increasing");
    return GXF_FAILURE;
  }
  if (prev_left_named_timestamp_ > left_named_timestamp->acqtime) {
    GXF_LOG_ERROR("Left named timestamp is not monotonically increasing");
    return GXF_FAILURE;
  }
  if (prev_right_named_timestamp_ > right_named_timestamp->acqtime) {
    GXF_LOG_ERROR("Right named timestamp is not monotonically increasing");
    return GXF_FAILURE;
  }

  prev_left_unnamed_timestamp_ = left_unnamed_timestamp->acqtime;
  prev_right_unnamed_timestamp_ = right_unnamed_timestamp->acqtime;
  prev_left_named_timestamp_ = left_named_timestamp->acqtime;
  prev_right_named_timestamp_ = right_named_timestamp->acqtime;

  RETURN_IF_ERROR(tx_left_camera_->publish(left_camera_entity));
  RETURN_IF_ERROR(tx_right_camera_->publish(right_camera_entity));

  return GXF_SUCCESS;
}

}  // namespace isaac
}  // namespace nvidia
