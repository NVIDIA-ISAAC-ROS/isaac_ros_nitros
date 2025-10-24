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
#pragma once

#include <string>
#include <utility>
#include <vector>

#include "gxf/core/gxf.h"

#include "common/assert.hpp"
#include "common/type_name.hpp"
#include "gxf/core/handle.hpp"
#include "gxf/core/parameter_parser_std.hpp"
#include "gxf/std/codelet.hpp"
#include "gxf/std/receiver.hpp"
#include "gxf/std/transmitter.hpp"

namespace nvidia {
namespace isaac {

// A codelet to sync camera messages with camera info messages. It will match the timestamp of a
// camera info message with the timestamp of the corresponding camera message. If the parameter
// for use_latest_camera_info is set to true, it will use the latest camera info message
// It will make sure to update the camera info message with the same timestamp as that of the
// camera message
class CameraInfoSynchronization : public gxf::Codelet {
 public:
  gxf_result_t registerInterface(gxf::Registrar* registrar) override;
  gxf_result_t start() override;
  gxf_result_t tick() override;
  // This function will drop oldest messages from either queues to keep the graph ticking.
  // It is needed when a user decides to use this codelet with a MultiMessageAvailableScheduling
  // term.
  gxf_result_t dropOldestMessagesIfQueueIsFull();
  // Gets latest camera info is in the queues and matches it with the image. It will update
  // timestamp of the camera info message to be the same as that of the camera message.
  gxf_result_t useLatestCameraInfo();

 private:
  gxf::Parameter<gxf::Handle<gxf::Receiver>> camera_message_rx_;
  gxf::Parameter<gxf::Handle<gxf::Receiver>> camera_info_message_rx_;
  gxf::Parameter<gxf::Handle<gxf::Transmitter>> camera_message_tx_;
  gxf::Parameter<gxf::Handle<gxf::Transmitter>> camera_info_message_tx_;
  gxf::Parameter<bool> use_latest_camera_info_;
  gxf::Parameter<bool> drop_old_messages_;
  gxf::Parameter<uint64_t> sync_policy_;
  // These parameters are used to store the timestamps of the camera and camera info messages
  // These are preallocated in the start function to not have memory access during runtime
  std::vector<int64_t> camera_acq_times_;
  std::vector<int64_t> camera_info_acq_times_;
};

}  // namespace isaac
}  // namespace nvidia
