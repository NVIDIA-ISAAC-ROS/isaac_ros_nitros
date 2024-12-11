// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef NVIDIA_GXF_APPLICATION_HPP_
#define NVIDIA_GXF_APPLICATION_HPP_

#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "common/assert.hpp"
#include "gxf/app/arg.hpp"
#include "gxf/app/config_parser.hpp"
#include "gxf/app/driver.hpp"
#include "gxf/app/extension_manager.hpp"
#include "gxf/app/segment.hpp"
#include "gxf/app/worker.hpp"
#include "gxf/core/expected_macro.hpp"
#include "gxf/core/gxf.h"

namespace nvidia {

namespace gxf {

template <typename T, typename... Args>
std::shared_ptr<T> create_app(Args&&... args) {
  auto app = std::make_shared<T>(std::forward<Args>(args)...);
  return app;
}

/**
 * @brief Enum representing the application mode of execution.
 *  An application can either have segments or entities in its
 *  compose() api.
 */
enum class ExecutionMode : int8_t {
  kUnset = 0,
  kSingleSegment,
  kMultiSegment,
  kDistributed,
};

/**
 * @brief Scaffolding layer to create applications imperatively. Users implement a virtual
 * compose() api where individual building blocks of an application is constructed, configured
 * and connected with each other.
 *
 */
class Application : public Segment {
 private:
  friend class Worker;
  friend class Driver;
 public:
  Application();

  ~Application();

  Application(const Application&) = delete;

  Application& operator=(const Application&) = delete;

  Application(Application&&) = delete;

  Application& operator=(Application&&) = delete;

  virtual void compose() {}

  /**
   * @brief Set segment config file
   *
   * @param file_path Absolute path to the yaml config file for segments
   * distributed execution
   * @return Expected<void> Success or error code
   */
  Expected<void> setConfig(const std::string& file_path);

  /**
   * @brief Set segment config file
   *
   * @param argc CLI argument count
   * @param argv CLI argument array, the second is config file path for segments
   * distributed execution
   * @return Expected<void> Success or error code
   */
  Expected<void> setConfig(int argc, char** argv);

  /**
   * @brief Creates a segment in plan with its own context. The graph for the segment
   * will be composed() after creation
   *
   * @tparam SegmentT Type of segment
   * @param name Name of the segment
   * @return std::shared_ptr<SegmentT> Shared pointer of the segment
   */
  template <typename SegmentT,
            typename = std::enable_if_t<!std::is_same_v<Segment, std::decay_t<SegmentT>>>>
  std::shared_ptr<SegmentT> createSegment(const char* name) {
    if (segments_plan_.find(name) != segments_plan_.end()) {
      GXF_LOG_ERROR("ProxySegment with name [%s] already exists. "
        "Segment names have to be unique", name);
      return nullptr;
    }
    std::shared_ptr<SegmentT> element = std::make_shared<SegmentT>();
    element->setName(name);
    segments_plan_.emplace(name, element);
    return element;
  }

  /**
   * @brief API to load extensions at runtime
   *
   * @param manifest path to manifest file with list of extensions
   * @return Expected<void> Success or error code
   */
  Expected<void> loadExtensionManifest(const char* manifest);

  /**
   * @brief Adds a UCX connection between two entities with many : many tx and rx
   *  Ucx Transmitter and Ucx Receiver components are added to the source and target entities in
   *  both of the segments
   * @param source Segment with the entity transmitting the message
   * @param target Segment with the entity receiving entity. A message available term is added
   *               along with the Ucx Receiver
   * @param port_maps Segment port map with entity and queue name to be used for connection.
   * @return Expected<void> Success if connection was added successfully, error code on failure
   */
  Expected<void> connect(SegmentPtr source, SegmentPtr target,
                         std::vector<SegmentPortPair> port_maps);

  /**
   * @brief Sets the severity level of the logs (corresponding to GXF_LOG_* logging macros)
   *  for a specific segment
   *
   *
   * @param name Name of the segment
   * @param severity a valid severity level as defined in `gxf_severity_t`. Logs corresponding to
   *                any level <= severity will be logged.
   * @return gxf_result_t On success the function returns GXF_SUCCESS.
   */
  gxf_result_t setSegmentSeverity(const char* name, gxf_severity_t severity);

  /**
   * @brief A blocking api to run the graph. If the application contains multiple segments,
   * each segment is launched asynchronously and this thread is blocked until each one of
   * the segments have finished execution. If the graph contains multiple entities,
   * then this thread is blocked until the graph execution is complete.
   * @return Expected<void> Success or error code
   */
  Expected<void> run();

  /**
   * @brief A non blocking api call to run an application. If the application contains multiple
   * segments, each segment is launched asynchronously.
   * @return Expected<void> Success or error code
   */
  Expected<void> runAsync();

  /**
   * @brief A non blocking api to stop all running running segments or entities.
   *
   * @return Expected<void> Success or error code
   */
  Expected<void> interrupt();

  /**
   * @brief A blocking API to waits until the graph execution has completed
   *
   * @return Expected<void> Success or error code
   */
  Expected<void> wait();

  /**
   * @brief In-place add a GraphWorker Component into Application's root context,
   * in which case Application's context should only hold and run GraphWorker or GraphDriver
  */
  template <typename... Args>
  Expected<void> setWorker(const std::string& name, Args... args) {
    if (worker_ != nullptr) {
      GXF_LOG_ERROR("Worker already created with name: %s", worker_->name().c_str());
      return Unexpected{GXF_FAILURE};
    }
    worker_ = std::make_shared<Worker>(this, name);

    std::vector<Arg> arg_list;
    if constexpr (sizeof...(args) > 0) { arg_list = parseArgsOfType<Arg>(args...); }
    // 1. parse worker its own port
    int32_t port = -1;
    Expected<Arg> arg_port = findArg(arg_list, kPort, GXF_PARAMETER_TYPE_INT32);
    if (!arg_port) {
      GXF_LOG_ERROR("Failed to find arg: %s when set GraphWorker[%s]", kPort, name.c_str());
      return Unexpected{arg_port.error()};
    }
    port = arg_port.value().as<int32_t>();
    // 2. parse driver_ip
    std::string driver_ip;
    Expected<Arg> arg_driver_ip = findArg(arg_list, kDriverIp, GXF_PARAMETER_TYPE_STRING);
    if (!arg_driver_ip) {
      GXF_LOG_ERROR("Arg 'driver_ip' is wrong when set GraphWorker[%s]", name.c_str());
      return Unexpected{arg_driver_ip.error()};
    }
    driver_ip = arg_driver_ip.value().as<std::string>();
    // 3. parse driver_port
    int32_t driver_port = -1;
    Expected<Arg> arg_driver_port = findArg(arg_list, kDriverPort, GXF_PARAMETER_TYPE_INT32);
    if (!arg_driver_port) {
      GXF_LOG_ERROR("Arg 'driver_port' is wrong when set GraphWorker[%s]", name.c_str());
      return Unexpected{arg_driver_port.error()};
    }
    driver_port = arg_driver_port.value().as<int32_t>();

    std::unordered_set<std::string> segment_names;
    for (const auto& arg : arg_list) {
      auto it = segments_.find(arg.key());
      if (it != segments_.end()) {
        GXF_LOG_DEBUG("Add Segment[name: %s] to GraphWorker[name: %s]",
          it->first.c_str(), name.c_str());
        segment_names.emplace(it->first);
      }
    }
    if (!segment_names.empty()) {
      GXF_LOG_ERROR("No segments selected in setWorker() arg list");
      return Unexpected{GXF_ARGUMENT_INVALID};
    }
    RETURN_IF_ERROR(worker_->setPort(port));
    RETURN_IF_ERROR(worker_->setDriverIp(driver_ip));
    RETURN_IF_ERROR(worker_->setDriverPort(driver_port));
    RETURN_IF_ERROR(worker_->setSegments(segment_names));
    return Success;
  }

  /**
   * @brief In-place add a GraphDriver Component into Application's root context,
   * in which case Application's context should only hold and run GraphWorker or GraphDriver
  */
  template <typename... Args>
  Expected<void> setDriver(const std::string& name, Args... args) {
    if (driver_ != nullptr) {
      GXF_LOG_ERROR("Driver already created with name: %s", driver_->name().c_str());
      return Unexpected{GXF_FAILURE};
    }
    driver_ = std::make_shared<Driver>(this, name);

    std::vector<Arg> arg_list;
    if constexpr (sizeof...(args) > 0) { arg_list = parseArgsOfType<Arg>(args...); }
    // parse driver its own port
    int32_t port = -1;
    Expected<Arg> arg_port = findArg(arg_list, kPort, GXF_PARAMETER_TYPE_INT32);
    if (!arg_port) {
      GXF_LOG_ERROR("Arg 'port' is wrong when set GraphDriver[%s]", name.c_str());
      return Unexpected{arg_port.error()};
    }
    port = arg_port.value().as<int32_t>();

    RETURN_IF_ERROR(driver_->setPort(port));
    return Success;
  }

  using Segment::connect;

 private:
  Expected<void> commitCompose();
  Expected<void> commitSegment(SegmentPtr segment, const char* name);
  Expected<void> commitConnect(SegmentPtr source, SegmentPtr target,
                         std::vector<SegmentPortPair> port_maps);
  Expected<void> checkConfiguration();
  Expected<void> activate();
  Expected<void> deactivate();
  Expected<void> finalize();
  Expected<void> setupCrashHandler();

  std::map<std::string, std::shared_ptr<Segment>> segments_;
  std::map<std::string, std::shared_ptr<Segment>> segments_plan_;
  std::unordered_set<SegmentConnection, SegmentConnection::Hash> segment_connections_;
  std::unordered_set<SegmentConnection, SegmentConnection::Hash> segment_connections_plan_;
  std::unordered_set<std::string> enabled_segments_;
  std::string name_;
  ExecutionMode mode_{ExecutionMode::kUnset};
  ExtensionManager extension_manager_;
  uint64_t next_ucx_port_ = 13337U;  // will be set to DEFAULT_UCX_PORT by constructor
  WorkerPtr worker_;
  DriverPtr driver_;
  std::unique_ptr<ConfigParser> config_parser_;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_APPLICATION_HPP_
