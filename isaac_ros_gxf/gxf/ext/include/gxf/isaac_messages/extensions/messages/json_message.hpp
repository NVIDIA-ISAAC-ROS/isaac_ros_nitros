/*
Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>

#include "engine/gems/sight/sop.hpp"
#include "gxf/core/entity.hpp"
#include "gxf/core/expected.hpp"
#include "gxf/std/timestamp.hpp"

namespace nvidia {
namespace isaac {

// Stores the Sop message and provides convenience views to it.
struct JsonMessageParts {
  // The message entity
  gxf::Entity entity;
  // Handle to the Sop instance for sight visualization
  gxf::Handle<::isaac::Json> json;
  // Timestamp
  gxf::Handle<gxf::Timestamp> timestamp;
};

// Creates message entity and attaches a Sop instance to it to be populated later for sight
// visualization.
gxf::Expected<JsonMessageParts> CreateJsonMessage(const char* tag_name, gxf_context_t context,
                                                  bool activate = true);

}  // namespace isaac
}  // namespace nvidia
