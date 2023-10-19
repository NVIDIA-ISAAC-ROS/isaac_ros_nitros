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

#include "gxf/core/entity.hpp"
#include "gxf/std/timestamp.hpp"

namespace nvidia {
namespace isaac {

// Correlated timestamp struct
// Nova has three seperate clocks that we must keep track of, the PTP hardware clock (PHC), the
// Timer's System Counter (TSC), and the system clock mainatined by the kernel (sys).
// This struct ontains two pairs of correlated timestamps, phc_val_ anc tsc_val are taken
// simultanously, and phc2_val and sys_val_ are taken simultanously.
struct CorrelatedTimestamps {
  // phc and tsc are captured at the same time
  int64_t phc_val;  // PTP Hardware Clock (PHC)
  int64_t tsc_val;  // Timer's System Counter (TSC)

  // phc2 and tsc are captures at the same time.
  int64_t phc2_val;  // PTP Hardware Clock (PHC), same clock as phc_val_ above, but taken at a
                     // different time
  int64_t sys_val;   // System Clock (Sys)

  // Latency of reading the PHC, slightly subtle, as this only applies to PHC2
  // Basically the simultanous reading of sys<->phc2 within nvpps does not account
  // The time it takes to read from the PHC, which can be several microseconds.
  // The value of phc2 published by the correlated_timestamp_driver already accounts for
  // This value, and phc_latency is provided to expose this value to GXF for sanity checking
  int64_t phc_latency;
};

// Stores the Correlated Timestamps message and provides views to it.
struct CorrelatedTimestampsMessageParts {
  // The message entity
  gxf::Entity entity;
  // View to the CorrelatedTimestamps struct
  gxf::Handle<CorrelatedTimestamps> correlated_timestamps;
  // GXF publication timestamp
  gxf::Handle<gxf::Timestamp> timestamp;
};

// Create a CorrelatedTimestampStruct and the owning entity.
gxf::Expected<CorrelatedTimestampsMessageParts> CreateCorrelatedTimestampMessage(
    gxf_context_t context);

// Create a Correlated timestamp struct from an existing entity
gxf::Expected<CorrelatedTimestampsMessageParts> GetCorrelatedTimestampMessage(gxf::Entity message);

}  // namespace isaac
}  // namespace nvidia
