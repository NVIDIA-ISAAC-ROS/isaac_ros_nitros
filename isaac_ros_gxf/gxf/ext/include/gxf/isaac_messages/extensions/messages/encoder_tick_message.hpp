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

#include "gxf/core/entity.hpp"
#include "gxf/core/expected.hpp"
#include "gxf/std/timestamp.hpp"

namespace nvidia {
namespace isaac {

struct EncoderTicksMessageParts {
  // Entity for the measurement
  gxf::Entity message;

  // Encoder data
  gxf::Handle<int32_t> left_ticks;   // ticks from the left wheel
  gxf::Handle<int32_t> right_ticks;  // ticks from the right wheel
  // Timestamp from the encoder, for example on the RMPLite this is a stamp on the CAN message
  // This can in general be in a different timebase than the system time
  gxf::Handle<int64_t> encoder_timestamp;

  // Timestamp of publishing and acquisition in system time
  gxf::Handle<gxf::Timestamp> timestamp;
};

gxf::Expected<EncoderTicksMessageParts> CreateEncoderTickMessage(gxf_context_t context);
gxf::Expected<EncoderTicksMessageParts> GetEncoderTickMessage(gxf::Entity message);

}  // namespace isaac
}  // namespace nvidia
