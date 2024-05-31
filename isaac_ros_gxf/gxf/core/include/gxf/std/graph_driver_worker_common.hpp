// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#ifndef NVIDIA_GXF_GRAPH_DRIVER_WORKER_COMMON_HPP_
#define NVIDIA_GXF_GRAPH_DRIVER_WORKER_COMMON_HPP_

#include <atomic>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "common/logger.hpp"
#include "gxf/core/expected.hpp"
#include "gxf/core/gxf.h"

namespace nvidia {
namespace gxf {

void parseIpAddress(const std::string& ip_address_port, std::string& ip_address, int& port);

void parseSegmentEntityComponentName(
  const std::string& segment_entity_component_name,
  std::string& segment_name,
  std::string& entity_name,
  std::string& component_name);

//
// JSON schema parsing
//
struct ComponentParam {
  struct ParamInfo {
    std::string key;
    std::string value;
    std::string value_type;
    static Expected<bool> strToBool(const std::string& str);
    static Expected<int32_t> strToInt32(const std::string& str);
    static Expected<uint32_t> strToUInt32(const std::string& str);
    static Expected<int64_t> strToInt64(const std::string& str);
    static Expected<uint64_t> strToUInt64(const std::string& str);
    static Expected<float> strToFloat32(const std::string& str);
    static Expected<double> strToFloat64(const std::string& str);
    static Expected<uint16_t> strToUInt16(const std::string& str);
  };
  std::string segment_name;
  std::string entity_name;
  std::string component_name;
  std::vector<ParamInfo> params;
  // Helpers
  std::string serialize() const {
    return segment_name + "." + entity_name + "." + component_name;
  }
};

struct ComponentInfo {
  std::string segment_name;
  std::string entity_name;
  std::string component_name;
  // Helpers
  std::string serialize() const {
    return segment_name + "." + entity_name + "." + component_name;
  }
  static ComponentInfo deserialize(const std::string& data) {
    std::istringstream stream(data);
    std::string segment;
    std::string entity;
    std::string component;
    std::getline(stream, segment, '.');
    std::getline(stream, entity, '.');
    std::getline(stream, component, '.');
    // Validate that all parts are present.
    if (segment.empty() || entity.empty() || component.empty()) {
      GXF_LOG_ERROR("Invalid data %s", data.c_str());
    }

    return {segment, entity, component};
  }
};

struct SegmentInfo {
  std::string segment_name;
  // key: "Segment_name/Entity_name/UcxRx_name"
  // value: "10.0.0.1:3000"
  std::map<std::string, std::string> ip_port_address_map;
};

struct WorkerInfo {
  std::string server_ip_address;
  std::string server_port;
  std::vector<SegmentInfo> segment_info_list;
  std::string ip_port() const {
    return server_ip_address + ":" + server_port;
  }
};

class GraphDriverWorkerParser {
 public:
  GraphDriverWorkerParser() = default;

  static Expected<std::vector<ComponentParam>>
  deserialize_onSetComponentParams(const std::string& payload);

  static Expected<std::string>
  serialize_onSetComponentParams(const std::vector<ComponentParam>& component_param_list);

  static Expected<WorkerInfo>
  deserialize_onRegisterGraphWorker(const std::string& payload);

  static Expected<std::string>
  serialize_onRegisterGraphWorker(const WorkerInfo& worker_info);
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_GRAPH_DRIVER_WORKER_COMMON_HPP_
