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

#include "gxf/std/codelet.hpp"
#include "gxf/std/receiver.hpp"
#include "gxf/std/transmitter.hpp"

namespace nvidia {
namespace isaac {

// This codelet checks if stereo frames timestamps are within a tolerance and
// will edit the right input timestamps to match the left input timestamp
// Some downstream applications expect matched stereo frames to have exactly the same timestamp
// even though in reality matched frames can differ by a few microseconds.
class StereoCameraSynchronizer : public gxf::Codelet {
  gxf_result_t registerInterface(gxf::Registrar* registrar) override;
  gxf_result_t start() override;
  gxf_result_t tick() override;
  gxf_result_t stop() override { return GXF_SUCCESS; }

 private:
  gxf::Parameter<gxf::Handle<gxf::Receiver>> rx_right_camera_;
  gxf::Parameter<gxf::Handle<gxf::Receiver>> rx_left_camera_;
  gxf::Parameter<gxf::Handle<gxf::Transmitter>> tx_right_camera_;
  gxf::Parameter<gxf::Handle<gxf::Transmitter>> tx_left_camera_;
  gxf::Parameter<int64_t> max_timestamp_diff_;
  int64_t prev_left_unnamed_timestamp_, prev_right_unnamed_timestamp_;
  int64_t prev_left_named_timestamp_, prev_right_named_timestamp_;
};

}  // namespace isaac
}  // namespace nvidia
