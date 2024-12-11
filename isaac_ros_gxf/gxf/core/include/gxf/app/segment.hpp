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

#ifndef NVIDIA_GXF_SEGMENT_HPP_
#define NVIDIA_GXF_SEGMENT_HPP_

#include <algorithm>
#include <cassert>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "common/assert.hpp"
#include "gxf/app/arg.hpp"
#include "gxf/app/arg_parse.hpp"
#include "gxf/app/entity_group.hpp"
#include "gxf/app/graph_entity.hpp"
#include "gxf/app/graph_utils.hpp"
#include "gxf/app/proxy_component.hpp"
#include "gxf/core/expected_macro.hpp"
#include "gxf/core/gxf.h"
#include "gxf/std/default_extension.hpp"
#include "gxf/std/resources.hpp"
#include "gxf/std/scheduler.hpp"
#include "gxf/std/scheduling_terms.hpp"

namespace nvidia {
namespace gxf {

/**
 * @brief A entity - entity connection specified using tx and rx component names
 * tx - transmitter component name
 * rx - receiver component name
 * The queue names should match the parameter keys of the codelet in the corresponding GraphEntity
 * for a successful connection between the two graph entities.
 */
typedef struct PortPair {
  PortPair(std::string tx, std::string rx) : tx(tx), rx(rx) {}
  std::string tx;
  std::string rx;
} PortPair;

/**
 * @brief An open port in an segment is specified using entity and queue component names.
 * SegmentPort name is in the format "<Entity Name>.<Queue Name>"
 * Entity Name - name of a graph entity created in the segment
 * Queue Name - tx or rx component name which should match the parameter keys of the codelet in
 *              the corresponding GraphEntity for a successful connection
 */

typedef struct SegmentPort {
  auto split_string(std::string port_name) {
    std::string entity;
    std::string queue;

    // Find the position of the dot
    size_t index = port_name.find('.');
    if (index != std::string::npos) {
      entity = port_name.substr(0, index);
      queue = port_name.substr(index + 1);
    } else {
      GXF_LOG_ERROR(
          "Invalid port name [%s]. Port names should follow [Entity.Queue] naming convention",
          port_name.c_str());
    }

    return std::make_pair(entity, queue);
  }
  auto split_string2(std::string port_name) {
    size_t first_pivot = port_name.find(".");
    size_t second_pivot = port_name.substr(first_pivot + 1).find(".");
    size_t third_pivot = port_name.find_last_of(".");
    if (first_pivot == std::string::npos
      || second_pivot == std::string::npos
      || third_pivot == std::string::npos) {
      GXF_LOG_ERROR(
          "Invalid port name [%s]. For port names with 3 dots, "
          "it should follow [Segment.Entity.Queue] naming convention",
          port_name.c_str());
    }
    std::string segment = port_name.substr(0, first_pivot);
    std::string entity = port_name.substr(first_pivot + 1, second_pivot);
    std::string queue = port_name.substr(third_pivot + 1);
    return std::make_tuple(segment, entity, queue);
  }

  std::string to_string() {
    if (!segment.empty()) {
      return segment + "." + entity + "." + queue;
    } else {
      return entity + "." + queue;
    }
  }

  SegmentPort(std::string name) {
    int count = std::count(name.begin(), name.end(), '.');
    if (count == 2) {
      std::tie(segment, entity, queue) = split_string2(name);
    } else {
      std::tie(entity, queue) = split_string(name);
    }
  }
  std::string segment;
  std::string entity;
  std::string queue;

  bool operator==(const SegmentPort& other) const {
    return segment == other.segment && entity == other.entity && queue == other.queue;
  }
} SegmentPort;


/**
 * @brief A segment - segment connection specified using the segment port info
 * SegmentPort name is in the format "<Entity Name>.<Queue Name>" where the queues are tx
 * or rx components.
 * tx - SegmentPort in the source segment
 * rx - SegmentPort in the sink segment
 *
 * The queue names should match the parameter keys of the codelet in the corresponding GraphEntity
 * for a successful connection between the two segments
 */
typedef struct SegmentPortPair {
  SegmentPortPair(std::string tx, std::string rx) : tx(tx), rx(rx) {}
  SegmentPort tx;
  SegmentPort rx;

  bool operator==(const SegmentPortPair& other) const {
    return tx == other.tx && rx == other.rx;
  }
} SegmentPortPair;

/**
 * @brief Enum representing the type of scheduler to be used in the application.
 * This enum is primarily used as an input to setScheduler api
 *
 */
enum class SchedulerType: int8_t {
  kGreedy = 0,
  kMultiThread,
  KEventBased,
};

const constexpr SchedulerType Greedy = SchedulerType::kGreedy;
const constexpr SchedulerType MultiThread = SchedulerType::kMultiThread;
const constexpr SchedulerType EventBased = SchedulerType::KEventBased;

/**
 * @brief Segment is a group of graph entities created in a single GXF runtime context. A segment
 * will have its own scheduler. Graph entities in a segment are connected with each other via
 * double buffer transmitter and receiver components. A segment can also be connected other
 * segments via ucx transmitters and receivers.
 *
 * Segments are created and managed by the nvidia::gxf::Application class.
 */
class Segment {
 public:
  Segment() = default;

  virtual ~Segment() = default;

  Segment(Segment&&) = delete;

  Segment& operator=(Segment&&) = delete;

  Segment(const Segment&) = delete;

  Segment& operator=(const Segment&) = delete;

  virtual void compose() {}

  /**
   * @brief Creates a graph entity with a codelet of type CodeletT along with a parameter pack of
   * Arg & ProxyComponent. The codelet component will be used to auto populate connection queues and
   * their corresponding scheduling terms. Args can be used to specify a variable list of components to be
   * created along with the codelet in the graph entity. Args can also be used to specify a variable list
   * of Arg type to update any parameter values of the codelet
   *
   * @tparam CodeletT Codelet component to be added in the Graph Entity
   * @tparam Args Pack of Arg or ProxyComponent objects
   * @param name Name of the graph entity
   * @param args Pack of Arg or ProxyComponent objects
   * @return GraphEntityPtr A newly created graph entity object with the requested components.
   */
  template <typename CodeletT, typename... Args>
  GraphEntityPtr makeEntity(const char* name, Args... args) {
    static_assert(std::is_base_of<Codelet, CodeletT>::value, "Requested type is not a Codelet");
    gxf_tid_t tid;
    gxf_result_t result = GxfComponentTypeId(context_, TypenameAsString<CodeletT>(), &tid);
    if (result == GXF_FACTORY_UNKNOWN_CLASS_NAME) {
      // dynamically register the new component
      auto result = registerCodelet<CodeletT>();
      if (result != GXF_SUCCESS) { return nullptr; }
    }

    auto entity = createGraphEntity(name);
    auto codelet = entity->addCodelet<CodeletT>(name);

    if constexpr (sizeof...(args) > 0) {
      // First create all proxy components
      std::vector<ProxyComponent> proxy_list = parseArgsOfType<ProxyComponent>(args...);
      for (auto proxy : proxy_list) { createFromProxy(proxy, entity); }

      // parse all codelet args
      std::vector<Arg> arg_list = parseArgsOfType<Arg>(args...);

      // filter all proxy component args and create them
      auto proxy_args = filterProxyComponents(arg_list);
      for (auto arg : proxy_args) {
        auto proxy = arg.as<ProxyComponent>();
        auto handle = createFromProxy(proxy, entity);
        applyArg(codelet, Arg(arg.key(), handle));
      }

      for (auto arg : arg_list) { applyArg(codelet, arg); }
    }

    return entity;
  }


  /**
   * @brief Creates a graph entity without a codelet and with a parameter pack of Arg
   * & ProxyComponent. Args can be used to specify a variable list of components to be
   * created along with the graph entity.
   *
   * @tparam Args Pack of Arg or ProxyComponent objects
   * @param name Name of the graph entity
   * @param args Pack of Arg or ProxyComponent objects
   * @return GraphEntityPtr A newly created graph entity object with the requested components.
   */
  template <typename... Args>
  GraphEntityPtr makeEntity(const char* name, Args... args) {
    auto entity = createGraphEntity(name);

    if constexpr (sizeof...(args) > 0) {
      // First create all proxy components
      std::vector<ProxyComponent> proxy_list = parseArgsOfType<ProxyComponent>(args...);
      for (auto proxy : proxy_list) { createFromProxy(proxy, entity); }
    }

    return entity;
  }

  EntityGroupPtr makeEntityGroup(const char* name,
    const std::vector<GraphEntityPtr>& entity_members = {}) {
    auto entity_group = createEntityGroup(name);
    entity_group->add(entity_members);

    return entity_group;
  }

  /**
   * @brief Creates a scheduling term of requested type and applies parameter component values
   * from a parameter pack of arguments. This api does not create the requested gxf native component.
   * A Proxy component value is returned which has the type info and arg list needed to create this
   * scheduling term. createFromProxy() api is used to create this component given any specific GraphEntity.
   *
   * @tparam T Type of GXF scheduling term. Type must be derived from nvidia::gxf::SchedulingTerm type.
   * @param name name of the component
   * @param args Parameter pack of arguments / parameter values to be applied to the component
   * @return ProxyComponent
   */
  template <typename T, typename... Args>
  ProxyComponent makeTerm(const char* name, Args... args) {
    static_assert(std::is_base_of<SchedulingTerm, T>::value,
                  "Requested type is not a SchedulingTerm");
    std::vector<Arg> arg_list;
    const std::string type_name(TypenameAsString<T>());
    if constexpr (sizeof...(args) > 0) { arg_list = parseArgsOfType<Arg>(args...); }
    return ProxyComponent(type_name, name, arg_list);
  }

  /**
   * @brief Creates a resource of requested type and applies parameter component values
   * from a parameter pack of arguments. This api does not create the requested gxf native component.
   * A Proxy component value is returned which has the type info and arg list needed to create this
   * resource. createFromProxy() api is used to create this component given any specific GraphEntity.
   *
   * @tparam T Type of GXF resource. Type must be derived from nvidia::gxf::ResourceBase type.
   * @param name name of the component
   * @param args Parameter pack of arguments / parameter values to be applied to the component
   * @return ProxyComponent
   */
  template <typename T, typename... Args>
  ProxyComponent makeResource(const char* name, Args... args) {
    // TODO(chandrahasj) Enable this check when allocator
    // types have been updated with resource base type
    // static_assert(std::is_base_of<ResourceBase, T>::value,
    //               "Requested type is not a Resource");
    std::vector<Arg> arg_list;
    const std::string type_name(TypenameAsString<T>());
    if constexpr (sizeof...(args) > 0) { arg_list = parseArgsOfType<Arg>(args...); }
    return ProxyComponent(type_name, name, arg_list);
  }

  /**
   * @brief Adds a clock component to the segment and applies parameter component values
   * from a parameter pack of arguments.
   *
   * @tparam T Type of GXF clock component. Type must be derived from nvidia::gxf::Clock type.
   * @param name Name of the clock component
   * @param args Parameter pack of arguments / parameter values to be applied to the component
   * @return Handle<Clock> Handle to newly created clock component. Null handle if component was not
   * created.
   */
  template <typename ClockT, typename... Args>
  Handle<Clock> setClock(const char* name, Args... args) {
    static_assert(std::is_base_of<Clock, ClockT>::value, "Requested type is not a Clock");
    if (!clock_entity_) { clock_entity_ = createGraphEntity("ClockEntity_" + name_); }

    auto clock = clock_entity_->add<ClockT>(name);
    std::vector<Arg> arg_list;
    if constexpr (sizeof...(args) > 0) { arg_list = parseArgsOfType<Arg>(args...); }
    for (auto arg : arg_list) { applyArg(clock, arg); }
    return clock;
  }

  /**
   * @brief Adds a scheduler component to the segment and applies parameter component values
   * from a parameter pack of arguments.
   *
   * @tparam T Type of GXF scheduler component. Type must be derived from nvidia::gxf::Scheduler type.
   * @param args Parameter pack of arguments / parameter values to be applied to the component
   * @return Handle<Scheduler> Handle to newly created scheduler component. Null handle if component was not
   * created.
   */
  template <SchedulerType schedulerType, typename... Args>
  Handle<Scheduler> setScheduler(Args... args) {
    std::vector<Arg> arg_list;
    if constexpr (sizeof...(args) > 0) { arg_list = parseArgsOfType<Arg>(args...); }

    return setScheduler(schedulerType, arg_list);
  }

  /**
   * @brief Add a scheduler to the segment based on the input SchedulerType enum. If the segment contains
   * a clock component, the same component will be reused to configure the scheduler. If no clock components
   * are found in the segment, a new RealTimeClock component will be added to the segment.
   *
   * @param scheduler Type of the scheduler to be added. One of kGreedy, kMultithread or kEventBased
   * @return Handle<Scheduler> Handle to newly created scheduler component. Null handle if component was not
   * created.
   */
  Handle<Scheduler> setScheduler(const SchedulerType& scheduler, std::vector<Arg> arg_list = {});

  /**
   * @brief Adds a double buffer queue based connection between two entities with
   * 1:1 tx and rx connectivity.
   *
   * @param source Origin graph entity for the connection
   * @param target Destination graph entity for the connection
   * @return Expected<void> On success the function returns Success
   */
  virtual Expected<void> connect(GraphEntityPtr& source, GraphEntityPtr& target);

  /**
   * @brief Adds a single double buffer queue based connection between two entities
   * with a port pair specified.
   *
   * @param source Origin graph entity for the connection
   * @param target Destination graph entity for the connection
   * @param port_pair Port pair containing info of connection to be created
   * @return Expected<void> On success the function returns Success
   */
  virtual Expected<void> connect(GraphEntityPtr& source, GraphEntityPtr& target,
                                 PortPair port_pair);


  /**
   * @brief Adds multiple double buffer queue based connections between two entities
   * with many : many tx and rx. Connections between two graph entities are created
   * sequentially.
   *
   * @param source Origin graph entity for the connection
   * @param target Destination graph entity for the connection
   * @param port_pairs List of port pairs containing info of connections to be created
   * @return Expected<void> On success the function returns Success
   */
  virtual Expected<void> connect(GraphEntityPtr& source, GraphEntityPtr& target,
                                 std::vector<PortPair> port_pairs);
  /**
   * @brief Get the Entity object with given name. A nullptr is returned if entity is
   * not found.
   *
   * @param name Name of the graph entity. API is Case sensitive.
   * @return std::shared_ptr<GraphEntity> Pointer to underlying graph entity object
   */
  std::shared_ptr<GraphEntity> getEntity(const char* name) {
    for (auto& [uid, entity] : entities_) {
      if (std::strcmp(name, entity->name()) == 0) { return entity; }
    }
    GXF_LOG_ERROR("Graph Entity [%s] not found", name);
    return nullptr;
  }

  /**
   * @brief Fetch the name of the segment
   *
   * @return const char* pointer to name of the segment
   */
  const char* name() const { return name_.c_str(); }

  /**
   * @brief Fetch the context of a segment
   *
   * @return gxf_context_t
   */
  gxf_context_t context() { return context_; }

  /**
   * @brief Activates all the graph entities in the segment
   *
   * @return Expected<void> On success the function returns Success
   */
  Expected<void> activate() {
    GXF_LOG_INFO("Activating segment [%s] ....", name_.c_str());
    gxf_result_t result = GxfGraphActivate(context_);
    return result == GXF_SUCCESS ? Success : Unexpected{result};
  }

  /**
   * @brief Deactivates all the graph entities in the segment
   *
   * @return Expected<void> On success the function returns Success
   */
  Expected<void> deactivate() {
    GXF_LOG_INFO("Deactivating segment [%s] ....", name_.c_str());
    gxf_result_t result = GxfGraphDeactivate(context_);
    return result == GXF_SUCCESS ? Success : Unexpected{result};
  }

  /**
   * @brief A blocking api to run the segment. This thread is blocked (sleeping)
   * until the segment execution is complete.
   *
   * @return Expected<void> On success the function returns Success
   */
  Expected<void> run() {
    GXF_LOG_INFO("Running segment [%s] ....", name_.c_str());
    gxf_result_t result = GxfGraphRun(context_);
    return result == GXF_SUCCESS ? Success : Unexpected{result};
  }

  /**
   * @brief A non blocking api to execute a segment. API returns immediately after
   * starting the segment execution. wait() can be used to wait until execution
   * has finished.
   * @return Expected<void> On success the function returns Success
   */
  Expected<void> runAsync() {
    GXF_LOG_INFO("Running segment [%s] ....", name_.c_str());
    gxf_result_t result = GxfGraphRunAsync(context_);
    return result == GXF_SUCCESS ? Success : Unexpected{result};
  }

  /**
   * @brief A non blocking api to stop a previously running segment. Segment is not guaranteed
   * to have stopped when this api returns. wait() can be used to wait until the execution
   * has finished.
   *
   * @return Expected<void> On success the function returns Success
   */
  Expected<void> interrupt() {
    GXF_LOG_INFO("Interrupting segment [%s] ....", name_.c_str());
    gxf_result_t result = GxfGraphInterrupt(context_);
    return result == GXF_SUCCESS ? Success : Unexpected{result};
  }

  /**
   * @brief A blocking API to wait until the segment execution has completed.
   *
   * @return gxf_result_t On success the function returns GXF_SUCCESS.
   */
  Expected<void> wait() {
    GXF_LOG_INFO("Waiting on segment [%s]....", name_.c_str());
    gxf_result_t result = GxfGraphWait(context_);
    return result == GXF_SUCCESS ? Success : Unexpected{result};
  }

  /**
   * @brief Sets the severity level of the logs (corresponding to GXF_LOG_* logging macros)
   *  for a segment
   *
   * @param severity a valid severity level as defined in `gxf_severity_t`. Logs corresponding to
   *                any level <= severity will be logged.
   * @return gxf_result_t On success the function returns GXF_SUCCESS.
   */
  gxf_result_t setSeverity(gxf_severity_t severity) { return GxfSetSeverity(context_, severity); }

  /**
   * @brief Saves the segment information containing entities, components and their corresponding
   * parameter values in a yaml representation
   *
   * @param filepath path to save the resulting graph yaml file
   * @return Expected<void> On success the function returns Success
   */
  Expected<void> saveToFile(const char* filepath) {
    gxf_result_t code = GxfGraphSaveToFile(context_, filepath);
    return ExpectedOrCode(code);
  }

  /**
   * @brief Loads parameters for graph entities composed in the segment / application.
   * YAML file follows the GXF graph specification
   *
   * @param filepath path to a valid parameters file
   * @return Expected<void> On success the function returns Success
   */
  Expected<void> loadParameterFile(const char* filepath) {
    gxf_result_t code = GxfGraphLoadFile(context_, filepath);
    return ExpectedOrCode(code);
  }

  /**
   * @brief Create a Network Context in the segment which can be used by UCX Connections
   * added in the application. A new graph entity with the name "NetworkContext" will be added
   * to the segment context with a UcxContext component and a corresponding entity and component
   * serializers
   *
   * @return Expected<void> On success the function returns Success
   */
  Expected<void> createNetworkContext();

  /**
   * @brief Creates a component in graph entity based on the type information from the ProxyComponent
   *
   * @param component A proxy component object
   * @param entity A pointer to graph entity to be used for creating the component
   * @return Handle<Component> Handle to newly created component
   */
  Handle<Component> createFromProxy(ProxyComponent& component, GraphEntityPtr& entity);

  /**
   * @brief This function is expected to be called by the application layer to assign a context to the
   * segment and a runtime extension for on the fly registration of components
   *
   * @param segment_context A valid GXF context to be assigned to the segment
   * @param name A valid name for the segment
   * @param runtime_ext Pointer to a GXF extension which can be used to register any components at runtime
   * @return Expected<void> On success the function returns Success
   */
  Expected<void> setup(gxf_context_t segment_context, const char* name,
                       std::shared_ptr<DefaultExtension> runtime_ext) {
    name_.assign(name);
    context_ = segment_context;
    runtime_ext_ = runtime_ext;
    is_setup_ = true;
    return Success;
  }

  Expected<void> setName(const char* name) {
    name_ = name;
    return Success;
  }

  Expected<void> checkConfiguration() { return Success; }

 protected:
  /**
   * @brief Creates a programmable graph entity with the given name
   *
   * @param name Name of the graph entity
   * @return GraphEntityPtr Pointer to newly created entity
   */
  GraphEntityPtr createGraphEntity(std::string name) {
    GXF_LOG_DEBUG("Creating graph entity [%s]", name.c_str());
    auto entity = std::make_shared<GraphEntity>();
    if (!entity->setup(context_, name.c_str())) {
      return nullptr;
    }
    entities_.emplace(entity->eid(), entity);
    return entity;
  }

  /**
   * @brief Creates a programmable graph entity group with the given name
   *
   * @param name Name of the graph entity
   * @return EntityGroupPtr Pointer to newly created entity group
   */
  EntityGroupPtr createEntityGroup(std::string name) {
    GXF_LOG_DEBUG("Creating graph entity group [%s]", name.c_str());
    auto entity_group = std::make_shared<EntityGroup>();
    if (!entity_group->setup(context_, name.c_str())) {
      return nullptr;
    }
    entity_groups_.emplace(entity_group->gid(), entity_group);
    return entity_group;
  }

  // Registers a new codelet type with the runtime extension
  template <typename T>
  gxf_result_t registerCodelet() {
    static_assert(std::is_base_of<Codelet, T>::value, "Requested type is not a Codelet");
    gxf_tid_t new_tid = generate_tid();
    auto result = runtime_ext_->add<T, Codelet>(new_tid, TypenameAsString<T>(),
                                                TypenameAsString<T>(), TypenameAsString<T>());
    if (!result) { return ToResultCode(result); }
    return GxfRegisterComponentInExtension(context_, new_tid, runtime_ext_->tid());
  }

  // Extension to register components on the fly
  std::shared_ptr<DefaultExtension> runtime_ext_;
  std::map<gxf_uid_t, GraphEntityPtr> entities_;
  std::map<gxf_uid_t, EntityGroupPtr> entity_groups_;
  GraphEntityPtr scheduler_entity_;
  GraphEntityPtr clock_entity_;
  GraphEntityPtr connect_entity_;
  GraphEntityPtr network_ctx_;
  gxf_context_t context_ = kNullContext;
  std::string name_;
  bool is_setup_{false};
};

typedef std::shared_ptr<Segment> SegmentPtr;

struct SegmentConnection {
  SegmentConnection(SegmentPtr source, SegmentPtr target, std::vector<SegmentPortPair> port_maps)
    : source(source), target(target), port_maps(port_maps) {}
  SegmentPtr source;
  SegmentPtr target;
  std::vector<SegmentPortPair> port_maps;

  bool operator==(const SegmentConnection& other) const {
    if (source != other.source || target != other.target) {
      return false;
    }
    if (port_maps.size() != other.port_maps.size()) {
      return false;
    }
    for (size_t i = 0; i < port_maps.size(); i++) {
      if (!(port_maps.at(i) == other.port_maps.at(i))) {
        return false;
      }
    }
    return true;
  }

  struct Hash {
    std::size_t operator()(const SegmentConnection& sc) const {
      std::size_t h1 = std::hash<SegmentPtr>{}(sc.source);
      std::size_t h2 = std::hash<SegmentPtr>{}(sc.target);
      std::size_t h3 = std::hash<std::size_t>{}(sc.port_maps.size());
      return h1 ^ h2 ^ h3;
    }
  };
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_SEGMENT_HPP_
