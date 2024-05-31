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
#ifndef NVIDIA_GXF_GRAPH_WORKER_HPP_
#define NVIDIA_GXF_GRAPH_WORKER_HPP_

#include <atomic>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include "gxf/std/gems/queue_thread/queue_thread.hpp"
#include "gxf/std/graph_driver_worker_common.hpp"
#include "gxf/std/ipc_client.hpp"
#include "gxf/std/ipc_server.hpp"
#include "gxf/std/system.hpp"

namespace nvidia {
namespace gxf {

// copy tid from ucx extension
static const gxf_tid_t ucx_transmitter_tid{0x58165d0378b74696, 0xb20071621f90aee7};

struct GraphSpec {
  std::string app_path;
  std::string param_path;
  std::string manifest_path;
  uint32_t severity = 3;  // default severity
};

/**
 * SegmentRunner, context to run each graph segment
*/
class SegmentRunner {
 public:
  using GxfSystemThread = QueueThread<std::string>;
  // YAML API compose graph
  SegmentRunner(const std::string& name, const GraphSpec& graph_spec)
    : name_(name), graph_spec_(graph_spec) {
    runner_thread_ = std::make_unique<GxfSystemThread>(
    std::bind(&SegmentRunner::asyncRunnerCallback, this, std::placeholders::_1, this),
    name);
  }
  SegmentRunner(const std::string& name, const GraphSpec& graph_spec,
    std::shared_ptr<GxfSystemThread> worker_thread)
    : SegmentRunner(name, graph_spec) {  // delegating to the first constructor
    worker_thread_ = worker_thread;
  }
  // C++ API compose graph
  SegmentRunner(const std::string& name, const gxf_context_t context)
    : name_(name), context_(context) {
    runner_thread_ = std::make_unique<GxfSystemThread>(
    std::bind(&SegmentRunner::asyncRunnerCallback, this, std::placeholders::_1, this),
    name);
  }
  SegmentRunner(const std::string& name, const gxf_context_t context,
    std::shared_ptr<GxfSystemThread> worker_thread)
    : SegmentRunner(name, context) {
    worker_thread_ = worker_thread;
  }
  // ~SegmentRunner();
  void stop();
  void wait();
  // combined/simplified gxf run sequence
  std::future<bool> asyncInitializeGxfGraph();
  std::future<bool> asyncActivateGxfGraph();
  std::future<bool> asyncRunGxfGraph();
  std::future<bool> runGxfGraph();
  std::future<bool> asyncDeactivateGxfGraph();
  std::future<bool> asyncDestroyGxfGraph();
  gxf_result_t setParameter(const std::string& entity_name, const std::string& comp_name,
    const std::string& key, const std::string& value, const std::string& value_type);
  Expected<SegmentInfo> createSegmentInfo(const std::string& worker_host_ip);

 private:
  std::unique_ptr<GxfSystemThread> runner_thread_;
  std::shared_ptr<GxfSystemThread> worker_thread_;
  bool asyncRunnerCallback(std::string event, SegmentRunner* self);
  struct Event {
    // GXF C API sequence event
    static constexpr const char* kCreateGxfContext = "kCreateContext";
    static constexpr const char* kLoadGxfManifest = "kLoadGxfManifest";
    static constexpr const char* kLoadGxfGraph = "kLoadGxfGraph";
    static constexpr const char* kActivateGxfGraph = "kActivateGxfGraph";
    static constexpr const char* kNonBlockingRunGxfGraph = "kNonBlockingRunGxfGraph";
    static constexpr const char* kBlockingRunGxfGraph = "kBlockingRunGxfGraph";
    static constexpr const char* kInterruptGxfGraph = "kInterruptGxfGraph";
    static constexpr const char* kDeactivateGxfGraph = "kDeactivateGxfGraph";
    static constexpr const char* kDestroyGxfGraph = "kDestroyGxfGraph";
  };
  const std::string name_;
  const GraphSpec graph_spec_;
  gxf_context_t context_ = kNullContext;
  gxf_context_t s_signal_context_ = kNullContext;
  std::mutex context_mutex_;

 private:
  // gxf wrappers, non thread safe to gxf_context
  gxf_result_t wrapCreateGxfConext();
  gxf_result_t wrapLoadGxfManifest();
  gxf_result_t wrapLoadGxfGraph();
  gxf_result_t wrapActivateGxfGraph();
  gxf_result_t wrapBlockingRunGxfGraph();
  gxf_result_t wrapNonBlockingRunGxfGraph();
  gxf_result_t wrapInterruptGxfGraph();
  gxf_result_t wrapDeactivateGxfGraph();
  gxf_result_t wrapDestroyGxfGraph();
};


/**
 * ECS layer Component, manages executing a set of segments
 * Works with GraphDriver as server-client pair
 *
 * Has event-based thread, IPC server, and IPC client.
 *
 * Segments to manage:
 * 1. Users to provide, via config or add segment context API
*/
class GraphWorker : public System {
 public:
  using GxfSystemThread = QueueThread<std::string>;
  GraphWorker() = default;

  gxf_result_t schedule_abi(gxf_uid_t eid) override;
  gxf_result_t unschedule_abi(gxf_uid_t eid) override;
  gxf_result_t runAsync_abi() override;
  gxf_result_t stop_abi() override;
  gxf_result_t wait_abi() override;
  gxf_result_t event_notify_abi(gxf_uid_t eid, gxf_event_t event) override;

  gxf_result_t registerInterface(Registrar* registrar) override;
  gxf_result_t initialize() override;
  gxf_result_t deinitialize() override;

  // C++ API exclusive
  Expected<void> addSegment(const std::string& name, const gxf_context_t context);

 private:
  // This ECS component layer
  /**
   * Member thread event-based entrance
  */
  bool asyncRunnerCallback(std::string event, GraphWorker* self);

  Parameter<std::map<std::string, GraphSpec>> graph_specs_;
  Parameter<int64_t> driver_reconnection_times_;
  std::map<std::string, std::unique_ptr<SegmentRunner>> segment_runners_;
  std::shared_ptr<GxfSystemThread> worker_thread_;
  size_t completed_segment_runner_nums_;
  struct Event {
    static constexpr const char* kInstantiateSegmentRunner = "kInstantiateSegmentRunner";
    static constexpr const char* kRegisterWorker = "kRegisterWorker";
    static constexpr const char* kCheckWorkComplete = "kCheckWorkComplete";
  };

  // Communication layer
  Parameter<Handle<gxf::IPCServer>> server_;
  Parameter<Handle<gxf::IPCClient>> client_;

  // Service layer
  Parameter<std::string> initialize_segments_uri_;
  Parameter<std::string> set_component_params_uri_;
  Parameter<std::string> activate_segments_uri_;
  Parameter<std::string> run_segments_uri_;
  Parameter<std::string> deactivate_segments_uri_;
  Parameter<std::string> destroy_segments_uri_;
  Parameter<std::string> stop_worker_uri_;

 private:
  // Services to handle request from the GraphDriver
  Expected<void> onInitializeSegments(const std::string& resource, const std::string& payload);
  Expected<void> onSetComponentParams(const std::string& resource, const std::string& payload);
  Expected<void> onActivateSegments(const std::string& resource, const std::string& payload);
  Expected<void> onRunSegments(const std::string& resource, const std::string& payload);
  Expected<void> onDeactivateSegments(const std::string& resource, const std::string& payload);
  Expected<void> onDestroySegments(const std::string& resource, const std::string& payload);
  Expected<void> onStopWorker(const std::string& resource, const std::string& payload);

  Expected<void> instantiateSegmentRunners();
  Expected<void> registerGraphWorker();
  Expected<void> checkComplete();
  std::unique_ptr<WorkerInfo> worker_info_;

  gxf_result_t stop_all_segments();

  // helper methods
  std::string getPrimaryIp();
  Expected<std::string> createWorkerInfo();

  friend class SegmentRunner;
};

//
// utils for Parameter<std::map<std::string, GraphSpec>>
//
template <>
struct ParameterParser<GraphSpec> {
  static Expected<GraphSpec> Parse(
    gxf_context_t context, gxf_uid_t component_uid, const char* key,
    const YAML::Node& node, const std::string& prefix) {
      if (!node.IsMap() || node.size() < 2) {
        return Unexpected{GXF_PARAMETER_PARSER_ERROR};
      }
      GraphSpec graph_spec;
      // mandatory fields
      graph_spec.app_path = node["app-path"].as<std::string>();
      graph_spec.manifest_path = node["manifest-path"].as<std::string>();
      // optional fields
      if (node["param-path"].IsDefined()) {  // Check if "param-path" is defined
        graph_spec.param_path = node["param-path"].as<std::string>();
      } else {
        graph_spec.param_path = "";
      }
      if (node["severity"].IsDefined()) {
        graph_spec.severity = node["severity"].as<uint32_t>();
      } else {
        graph_spec.severity = 3;
      }
      return graph_spec;
    }
};

template <>
struct ParameterWrapper<GraphSpec> {
  // Wrap the value to a YAML::Node instance
  static Expected<YAML::Node> Wrap(
    gxf_context_t context,
    const GraphSpec& value) {
    YAML::Node node;
    // mandatory fields
    node["app-path"] = value.app_path;
    node["manifest-path"] = value.manifest_path;
    // optional fields
    if (!value.param_path.empty()) {
      node["param-path"] = value.param_path;
    }
    if (value.severity >= 0 && value.severity <= 4) {
      node["severity"] = value.severity;
    }
    return node;
  }
};

template <>
struct ParameterParser<std::map<std::string, GraphSpec>> {
  static Expected<std::map<std::string, GraphSpec>> Parse(
    gxf_context_t context, gxf_uid_t component_uid, const char* key,
    const YAML::Node& node, const std::string& prefix) {
      if (!node.IsMap()) {
        return Unexpected{GXF_PARAMETER_PARSER_ERROR};
      }
      std::map<std::string, GraphSpec> segment_runners;
      for (const auto& p : node) {
        const std::string k = p.first.as<std::string>();
        const auto maybe = ParameterParser<GraphSpec>::Parse(
          context, component_uid, k.c_str(), node[k], prefix);
        if (!maybe) {
          return ForwardError(maybe);
        }
        segment_runners[k] = maybe.value();
      }
      return segment_runners;
    }
};

template <>
struct ParameterWrapper<std::map<std::string, GraphSpec>> {
  // Wrap the value to a YAML::Node instance
  static Expected<YAML::Node> Wrap(
    gxf_context_t context,
    const std::map<std::string, GraphSpec>& value) {
    YAML::Node node(YAML::NodeType::Map);
    for (auto &i : value) {
      auto maybe = ParameterWrapper<GraphSpec>::Wrap(context, i.second);
      if (!maybe) {
        return ForwardError(maybe);
      }
      node[i.first] = maybe.value();
    }
    return node;
  }
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_GRAPH_WORKER_HPP_
