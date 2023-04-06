/*
 * SPDX-FileCopyrightText: Copyright (c) 2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef NVIDIA_COMMON_NVTX_HELPER_HPP_
#define NVIDIA_COMMON_NVTX_HELPER_HPP_

#include <string>

#include "nvtx3/nvToolsExt.h"

// Helper functions to create named color events for NVTX profiling

nvtxEventAttributes_t CreateGreenEvent(const std::string& message) {
  nvtxEventAttributes_t eventAttrib = {0};
  eventAttrib.version = NVTX_VERSION;
  eventAttrib.size = NVTX_EVENT_ATTRIB_STRUCT_SIZE;
  eventAttrib.colorType = NVTX_COLOR_ARGB;
  eventAttrib.color = 0xFF76b900;  // NVIDIA GREEN
  eventAttrib.messageType = NVTX_MESSAGE_TYPE_ASCII;
  eventAttrib.message.ascii = message.c_str();

  return eventAttrib;
}

nvtxEventAttributes_t CreateRedEvent(const std::string& message) {
  nvtxEventAttributes_t eventAttrib = {0};
  eventAttrib.version = NVTX_VERSION;
  eventAttrib.size = NVTX_EVENT_ATTRIB_STRUCT_SIZE;
  eventAttrib.colorType = NVTX_COLOR_ARGB;
  eventAttrib.color = 0xFFFe2712;  // Red
  eventAttrib.messageType = NVTX_MESSAGE_TYPE_ASCII;
  eventAttrib.message.ascii = message.c_str();

  return eventAttrib;
}


nvtxEventAttributes_t CreateBlackEvent(const std::string& message) {
  nvtxEventAttributes_t eventAttrib = {0};
  eventAttrib.version = NVTX_VERSION;
  eventAttrib.size = NVTX_EVENT_ATTRIB_STRUCT_SIZE;
  eventAttrib.colorType = NVTX_COLOR_ARGB;
  eventAttrib.color = 0xFF010203;  // Rich black
  eventAttrib.messageType = NVTX_MESSAGE_TYPE_ASCII;
  eventAttrib.message.ascii = message.c_str();

  return eventAttrib;
}

nvtxEventAttributes_t CreateBlueEvent(const std::string& message) {
  nvtxEventAttributes_t eventAttrib = {0};
  eventAttrib.version = NVTX_VERSION;
  eventAttrib.size = NVTX_EVENT_ATTRIB_STRUCT_SIZE;
  eventAttrib.colorType = NVTX_COLOR_ARGB;
  eventAttrib.color = 0xFF66a1fe;  // Light Blue
  eventAttrib.messageType = NVTX_MESSAGE_TYPE_ASCII;
  eventAttrib.message.ascii = message.c_str();

  return eventAttrib;
}

#endif  // NVIDIA_COMMON_NVTX_HELPER_HPP_
