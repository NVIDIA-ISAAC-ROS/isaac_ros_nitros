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
#ifndef NVIDIA_GXF_GRAPH_DRIVER_HPP_
#define NVIDIA_GXF_GRAPH_DRIVER_HPP_

#include <memory>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "gxf/std/gems/queue_thread/queue_thread.hpp"
#include "gxf/std/graph_driver_worker_common.hpp"
#include "gxf/std/ipc_client.hpp"
#include "gxf/std/ipc_server.hpp"
#include "gxf/std/system.hpp"

namespace nvidia {
namespace gxf {

/**
 * ECS layer Component, resolving segments connection addresses
 * Works with GraphWorker as server-client pair
 *
 * Has event-based thread, IPC server, and IPC client
 *
 * Segments connection graph:
 * 1. Users to provide, via config or add connection API
 *
 * Resolve address of each segment pair:
 * 1. Listen to remote GraphWorkers that each runs one or more segments.
 * 2. Register all GraphWorkers until all segments in static graph are discovered
 * 3. Resolve connection address between each segment pair
 * 4. Send result address to each target segment via GraphWorker that manages the segment
*/
class GraphDriver : public System {
 public:
  using ConnectionMap = std::unordered_map<std::string, std::string>;
  using GxfSystemThread = QueueThread<std::string>;
  GraphDriver() = default;

  gxf_result_t schedule_abi(gxf_uid_t eid) override;
  gxf_result_t unschedule_abi(gxf_uid_t eid) override;
  gxf_result_t runAsync_abi() override;
  gxf_result_t stop_abi() override;
  gxf_result_t wait_abi() override;
  gxf_result_t event_notify_abi(gxf_uid_t eid, gxf_event_t event) override;

  gxf_result_t registerInterface(Registrar* registrar) override;
  gxf_result_t initialize() override;

  // C++ API exclusive
  Expected<void> addSegmentConnection(const std::string& source, const std::string& target);

 private:
  /**
   * Member thread event-based entrance
  */
  bool asyncRunnerCallback(std::string event, GraphDriver* self);

  /**
   * Service to register a remote GraphWorker
  */
  Expected<void> onRegisterGraphWorker(const std::string& resource, const std::string& payload);

  /**
   * Service to record completion of each GraphWorker
   * then finally gently tear down this GraphDriver
  */
  Expected<void> onGraphWorkerComplete(const std::string& resource, const std::string& payload);

  /**
   * After discover all segments from remote GraphWorkers, resolve segments connection
   * then send remote request to update connection addresses
  */
  Expected<void> resolveConnections();

  /**
   * After update segments connection in each segment, they're ready to execute
   * send remote request to all GraphWorkers to run the segments
  */
  Expected<void> executeWorkers();

  /**
   * Send remote request to all GraphWorkers to deactivate the segments
  */
  Expected<void> deactivateWorkers();

  /**
   * Send remote request to all GraphWorkers to stop the segments
  */
  Expected<void> stopWorkers();

 private:
  // Communication layer
  Parameter<Handle<IPCServer>> server_;
  Parameter<Handle<IPCClient>> client_;
  Parameter<std::vector<ConnectionMap>> list_of_connections_;
  ConnectionMap connections_;
  ConnectionMap reverse_connections_;
  std::set<std::string> segment_names_;
  std::unordered_map<std::string, std::vector<std::string>> workers_;
  std::set<std::string> completed_workers_;
  std::unique_ptr<GxfSystemThread> driver_thread_;
  std::set<std::string> requested_segment_names_;
  std::unordered_map<std::string, std::string> segment_ip_address_;

 private:
  struct Event {
    static constexpr const char* kNotInitialized = "kNotInitialized";
    static constexpr const char* kResolveConnections = "kResolveConnections";
    static constexpr const char* kExecuteWorkers = "kExecuteWorkers";
    static constexpr const char* kDeactivateWorkers = "kDeactivateWorkers";
    static constexpr const char* kStopWorkers = "kStopWorkers";
  };
};

/**
 * Utils
*/
template <>
struct ParameterParser<GraphDriver::ConnectionMap> {
  static Expected<GraphDriver::ConnectionMap> Parse(
    gxf_context_t context, gxf_uid_t component_uid, const char* key,
    const YAML::Node& node, const std::string& prefix) {
      if (!node.IsMap()) {
        return Unexpected{GXF_PARAMETER_PARSER_ERROR};
      }
      GraphDriver::ConnectionMap connections;
      for (const auto& p : node) {
        const std::string k = p.first.as<std::string>();
        const auto maybe = ParameterParser<std::string>::Parse(
          context, component_uid, k.c_str(), node[k], prefix);
        if (!maybe) {
          return ForwardError(maybe);
        }
        connections[k] = maybe.value();
      }
      return connections;
    }
};


template <>
struct ParameterWrapper<GraphDriver::ConnectionMap> {
  // Wrap the value to a YAML::Node instance
  static Expected<YAML::Node> Wrap(
    gxf_context_t context,
    const GraphDriver::ConnectionMap& value) {
    YAML::Node node(YAML::NodeType::Map);
    for (auto &i : value) {
      auto maybe = ParameterWrapper<std::string>::Wrap(context, i.second);
      if (!maybe) {
        return ForwardError(maybe);
      }
      node[i.first] = maybe.value();
    }
    return node;
  }
};

template<>
struct ParameterWrapper<std::vector<GraphDriver::ConnectionMap>> {
  static Expected<YAML::Node> Wrap(
    gxf_context_t context,
    const std::vector<GraphDriver::ConnectionMap>& value) {
      YAML::Node node(YAML::NodeType::Sequence);
      for (auto &h : value) {
        auto maybe = ParameterWrapper<GraphDriver::ConnectionMap>::Wrap(context, h);
        if (!maybe) {
          return Unexpected{maybe.error()};
        }
        node.push_back(maybe.value());
      }
    return node;
  }
};

template <>
struct ParameterParser<std::vector<GraphDriver::ConnectionMap>> {
  static Expected<std::vector<GraphDriver::ConnectionMap>> Parse(
                                        gxf_context_t context, gxf_uid_t component_uid,
                                        const char* key, const YAML::Node& node,
                                        const std::string& prefix) {
    if (!node.IsSequence()) {
      const char* component_name = "UNKNOWN";
      GxfParameterGetStr(context, component_uid, kInternalNameParameterKey, &component_name);
      GXF_LOG_ERROR("Parameter '%s' in component '%s' must be a vector", key, component_name);
      return Unexpected{GXF_PARAMETER_PARSER_ERROR};
    }
    std::vector<GraphDriver::ConnectionMap> result(node.size());
    for (size_t i = 0; i < node.size(); i++) {
      const auto maybe = ParameterParser<GraphDriver::ConnectionMap>::Parse(
        context, component_uid, key, node[i], prefix);
      if (!maybe) {
        return ForwardError(maybe);
      }
      result[i] = std::move(maybe.value());
    }
    return result;
  }
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_GRAPH_DRIVER_HPP_
