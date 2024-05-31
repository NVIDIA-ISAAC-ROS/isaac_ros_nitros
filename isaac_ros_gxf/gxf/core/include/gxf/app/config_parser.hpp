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
#ifndef NVIDIA_GXF_GRAPH_CONFIG_PARSER_HPP_
#define NVIDIA_GXF_GRAPH_CONFIG_PARSER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "gxf/core/expected.hpp"
#include "yaml-cpp/yaml.h"

namespace YAML { class Node; }

namespace nvidia {
namespace gxf {

// identifiers
static constexpr const char* kSegmentConfig = "segment_config";
static constexpr const char* kMemberType = "member";
static constexpr const char* kMemberParam = "parameters";
static constexpr const char* kEnabledSegments = "enabled_segments";
static constexpr const char* kWorker = "worker";
static constexpr const char* kDriver = "driver";
// attributes
static constexpr const char* kName = "name";
static constexpr const char* kEnabled = "enabled";
static constexpr const char* kPort = "port";
static constexpr const char* kDriverIp = "driver_ip";
static constexpr const char* kDriverPort = "driver_port";
static constexpr const char* kRemoteAccess = "remote_access";
static constexpr const char* kServerIpAddress = "server_ip_address";

/**
 * @brief Read and parse YAML config files
 *
 */
class ConfigParser {
 public:
  struct SegmentConfig {
    struct EnabledSegments {
      std::vector<std::string> names;
    };
    struct Worker {
      bool enabled = true;
      std::string name;
      int32_t port = 0;
      std::string driver_ip;
      int32_t driver_port = 0;
    };
    struct Driver {
      bool enabled = false;
      std::string name;
      int32_t port = 0;
    };
    EnabledSegments enabled_segments;
    Worker worker;
    Driver driver;
    bool enable_all_segments = true;
  };
  ConfigParser() {
    segment_control_ = std::make_shared<SegmentConfig>();
  }
  /**
   * Main API
  */
  Expected<void> setFilePath(const std::string& file_path);
  Expected<void> setFilePath(int argc, char** argv);
  Expected<std::shared_ptr<SegmentConfig>> getSegmentConfig();

 private:
  std::string file_path_;
  std::shared_ptr<SegmentConfig> segment_control_;

 private:
  std::string getExecutablePath();
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_GRAPH_CONFIG_PARSER_HPP_
