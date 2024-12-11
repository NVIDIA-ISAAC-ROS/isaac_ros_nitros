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

#ifndef NVIDIA_GXF_GRAPH_ENTITY_HPP_
#define NVIDIA_GXF_GRAPH_ENTITY_HPP_

#include <cassert>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "gxf/app/arg.hpp"
#include "gxf/app/arg_parse.hpp"
#include "gxf/core/entity.hpp"
#include "gxf/core/gxf.h"
#include "gxf/std/clock.hpp"
#include "gxf/std/codelet.hpp"
#include "gxf/std/receiver.hpp"
#include "gxf/std/resources.hpp"
#include "gxf/std/scheduling_terms.hpp"
#include "gxf/std/transmitter.hpp"

namespace nvidia {
namespace gxf {

/**
 * @brief  A wrapper over nvidia::gxf::Entity to manage a programmable
 * graph entity.
 *
 */
class GraphEntity {
 public:
  GraphEntity() = default;

  ~GraphEntity() = default;

  GraphEntity(const GraphEntity&) = delete;

  GraphEntity& operator=(const GraphEntity&) = delete;

  /**
   * @brief Creates a programmable entity with the runtime context and sets its name
   *
   * @param context A valid GXF context
   * @param name Name of the graph entity
   */
  Expected<void> setup(gxf_context_t context, const char* name) {
    const GxfEntityCreateInfo entity_create_info = {name, GXF_ENTITY_CREATE_PROGRAM_BIT};
    auto result = GxfCreateEntity(context, &entity_create_info, &eid_);
    if (result != GXF_SUCCESS) {
      GXF_LOG_ERROR("Failed to create entity [%s] with error %s", name, GxfResultStr(result));
      return Unexpected{result};
    }
    GxfEntityRefCountInc(context, eid_);
    Entity::Own(context, eid_).map([&](Entity entity) { entity_ = std::move(entity); });
    return Success;
  }

  /**
   * @brief Creates a generic component of type T and sets the parameter values from Args
   * pack of args.
   *
   * Transmitters, Receivers, Clocks and Scheduling Term component names have to be unique.
   *
   * @tparam T Type of component. Must be derived from nvidia::gxf::Component
   * @param name Name of the component
   * @param args Args must be of type Arg
   * @return Handle<T> Handle to newly created component. Null handle if component was not created.
   */
  template <typename T, typename... Args>
  Handle<T> add(const char* name = nullptr, Args... args) {
    std::vector<Arg> arg_list;
    if constexpr (sizeof...(args) > 0) { arg_list = parseArgsOfType<Arg>(args...); }
    return add<T>(name, std::move(arg_list));
  }

  /**
   * @brief Creates a generic component of type T and sets the parameter values from arg_list.
   *
   * Transmitters, Receivers, Clocks and Scheduling Term component names have to be unique.
   *
   * @tparam T Type of component. Must be derived from nvidia::gxf::Component
   * @param name Name of the component
   * @param arg_list vector of Arg used for initializing the component's parameters.
   * @return Handle<T> Handle to newly created component. Null handle if component was not created.
   */
  template <typename T>
  Handle<T> add(const char* name, std::vector<Arg> arg_list) {
    auto maybe_component = addComponent<T>(name, arg_list);
    if (!maybe_component) { return Handle<T>::Null(); }
    return maybe_component.value();
  }

  /**
   * @brief Finds all components of given type. Returns an empty vector if component is not found
   *
   * @tparam T Type of component to search for
   * @return FixedVector<Handle<T>, N> List of handles to components of the same type
   */
  template <typename T, size_t N = kMaxComponents>
  FixedVector<Handle<T>, N> findAll() const {
    auto maybe_result = entity_.findAll<T, N>();
    return maybe_result ? maybe_result.value() : FixedVector<Handle<T>, N>();
  }

  /**
   * @brief Gets a component by type and name. Returns null handle if no such component.
   *
   * @tparam T Type of component to search for
   * @param name Name of the component to look for
   * @return Handle<T> Handle to component, if component is found. Null handle if
   * no such component.
   */
  template <typename T>
  Handle<T> get(const char* name = nullptr) const {
    auto maybe_handle = entity_.get<T>(name);
    if (!maybe_handle) {
      GXF_LOG_WARNING("Failed to find component [%s] with name [%s] in entity [%s] with error %s",
                      TypenameAsString<T>(), name, entity_.name(),
                      GxfResultStr(maybe_handle.error()));
      return Handle<T>::Null();
    }
    return maybe_handle.value();
  }

  /**
   * @brief Get a component by type and name. Returns an Unexpected in the case that the
   * component is not found. Unlike `get` no error is logged if a component is not found.
   *
   * @tparam T Type of component to search for
   * @param name Name of the component to look for
   * @return Expected<Handle<Component>> Handle to the component, if component is found.
   * Otherwise, an Unexpected is returned.
   */
  template <typename T>
  Expected<Handle<T>> try_get(const char* name = nullptr) const {
    auto maybe_handle = entity_.get<T>(name);
    if (!maybe_handle) { return Unexpected{GXF_ENTITY_COMPONENT_NOT_FOUND}; }
    return maybe_handle.value();
  }

  /**
   * @brief Gets a component by type and name. Returns null handle if no such component.
   *
   * @param type_name Fully qualified C++ type name of the component to search for
   * @param name Name of the component to look for
   * @return Handle<Component> Handle to component, if component is found. Null handle if
   * no such component.
   */
  Handle<Component> get(const char* type_name, const char* name = nullptr) const;

  /**
   * @brief Get a component by type and name. Returns an Unexpected in the case that the
   * component is not found. Unlike `get` no error is logged if a component is not found.
   *
   * @param type_name Fully qualified C++ type name of the component to search for
   * @param name Name of the component to look for
   * @return Expected<Handle<Component>> Handle to the component, if component is found.
   * Otherwise, an Unexpected is returned.
   */
  Expected<Handle<Component>> try_get(const char* type_name, const char* name = nullptr) const;

  /**
   * @brief Adds a codelet of type T with given name and sets the parameter values from Args
   *
   * @tparam T Type of codelet. Must be derived from nvidia::gxf::Codelet
   * @param name name of the codelet
   * @param args Args must be of type Arg
   * @return Handle<T> Handle to newly created component
   */
  template <typename T, typename... Args>
  Handle<T> addCodelet(const char* name = nullptr, Args... args) {
    static_assert(std::is_base_of<Codelet, T>::value, "Requested type is not a codelet");
    if (!codelet_.is_null()) {
      GXF_LOG_ERROR("Graph Entity is already configured with a codelet [%s]", codelet_->name());
      return Handle<T>::Null();
    }

    auto codelet = add<T>(name, args...);
    if (!codelet) { return codelet; }

    codelet_ = codelet;
    return codelet;
  }

  /**
   * @brief Adds a codelet with a given C++ type name.
   *
   * @param type_name The fully qualified C++ type name of the codelet component
   * @param name Name of the codelet
   * @param arg_list Arguments for the codelet
   * @return Handle<Codelet> Handle to newly created codelet component
   */
  Handle<Codelet> addCodelet(const char* type_name, const char* name = nullptr,
                             const std::vector<Arg>& arg_list = {});

  /**
   * @brief Adds a component with a given C++ type name.
   *
   * @param type_name The fully qualified C++ type name of the component
   * @param name Name of the component
   * @param arg_list Arguments for the component
   * @return Handle<Component> Handle to newly created component
   */
  Handle<Component> addComponent(const char* type_name, const char* name = nullptr,
                                 const std::vector<Arg>& arg_list = {});

  /**
   * @brief Adds a component of Clock type T and sets the parameter values from Args
   *
   * @tparam T Type of clock. Must be derived from nvidia::gxf::Clock.
   * @param name Name of the clock
   * @param args Args must be of type Arg
   * @return Handle<T> Handle to newly created clock component
   */
  template <typename T, typename... Args>
  Handle<T> addClock(const char* name = nullptr, Args... args) {
    static_assert(std::is_base_of<Clock, T>::value, "Requested type is not a clock");
    auto clock = add<T>(name, args...);
    return clock;
  }

  /**
   * @brief Adds a clock component with a given C++ type name.
   *
   * @param type_name The fully qualified C++ type name of the clock
   * @param name Name of the clock
   * @param arg_list Arguments for the clock component
   * @return Handle<Clock> Handle to newly created clock
   */
  Handle<Clock> addClock(const char* type_name, const char* name = nullptr,
                         const std::vector<Arg>& arg_list = {});

  /**
   * @brief Get the Clock object from a graph entity. Returns null handle if no clock component
   * has been created yet. Returns the first clock if no component name is provided. If name is
   * provided, exact instance of the clock is returned if found else a Null handle.
   *
   * @param name Name of the clock component to lookup
   * @return Handle<Clock>
   */
  Handle<Clock> getClock(const char* name = nullptr);

  /**
   * @brief Adds a component of SchedulingTerm type T and sets the parameter values from Args
   *
   * @tparam T Type of SchedulingTerm. Must be derived from nvidia::gxf::SchedulingTerm
   * @param name name of the scheduling term
   * @param args Args must be of type Arg
   * @return Handle<T> Handle to newly created scheduling term component
   */
  template <typename T, typename... Args>
  Handle<T> addSchedulingTerm(const char* name = nullptr, Args... args) {
    static_assert(std::is_base_of<SchedulingTerm, T>::value,
                  "Requested type is not a SchedulingTerm");
    auto term = add<T>(name, args...);
    return term;
  }

  /**
   * @brief Adds a scheduling term component with a given C++ type name.
   *
   * @param type_name The fully qualified C++ type name of the scheduling term
   * @param name Name of the scheduling term
   * @return Handle<SchedulingTerm> Handle to newly created scheduling term
   */
  Handle<SchedulingTerm> addSchedulingTerm(const char* type_name, const char* name = nullptr,
                                           const std::vector<Arg>& arg_list = {});

  /**
   * @brief Adds a component of Transmitter type T with name and sets the parameter values from
   * Args
   *
   * Name of the transmitter should match the parameter name of the underlying codelet
   * The name of the transmitter component is updated based on the parameter rank info.
   * A Downstream receptive scheduling term is also added to monitor the transmitter component
   *
   * If codelet parameter is a scalar, name of the transmitter is also same as the parameter key
   * Parameter<Handle<Transmitter>> | name - "key"
   *
   * If codelet parameter is a vector/array, the name of the transmitter component is key_%d where
   * 'd' is the index of this transmitter in the codelet parameter.
   * Parameter<Vector<Handle<Transmitter>> | name - "key_0", "key_1", "key_2"
   *
   * @tparam T Type of Transmitter. Must be derived from nvidia::gxf::Transmitter
   * @param name Name of the transmitter component
   * @param omit_term Boolean flag controlling whether or not a default downstream receptive
   *                  scheduling term is added. If true, no scheduling term is added.
   * @param args Args must be of type Arg
   * @return Handle<T> Handle to newly created transmitter component
   */
  template <typename T, typename... Args>
  Handle<T> addTransmitter(const char* name, bool omit_term = false, Args... args) {
    static_assert(std::is_base_of<Transmitter, T>::value, "Requested type is not a Transmitter");
    auto maybe_tx_name = formatTxName(name);
    if (!maybe_tx_name) { return Handle<T>::Null(); }
    auto tx_name = maybe_tx_name.value();

    auto tx = add<T>(tx_name.c_str(), args...);
    if (!tx) { return tx; }

    if (!omit_term) {
      // auto adds a downstream receptive scheduling term
      auto term = this->add<DownstreamReceptiveSchedulingTerm>(tx_name.c_str());
      term->setTransmitter(tx);
    }

    std::string full_name = std::string(this->name()) + "/" + tx_name;
    auto result = updatePort(name, full_name);
    if (!result) {
      GXF_LOG_ERROR("Failed to add Transmitter [%s] with error [%s]", tx_name.c_str(),
                    GxfResultStr(result.error()));
    }

    return tx;
  }

  /**
   * @brief Adds a component of Transmitter of the corresponding type_name.
   * Name of the transmitter should match the parameter name of the underlying codelet
   * The name of the transmitter component is updated based on the parameter rank info.
   *
   * If codelet parameter is a scalar, name of the transmitter is also same as the parameter key
   * Parameter<Handle<Transmitter>> | name - "key"
   *
   * If codelet parameter is a vector/array, the name of the transmitter component is key_%d where
   * 'd' is the index of this transmitter in the codelet parameter.
   * Parameter<Vector<Handle<Transmitter>> | name - "key_0", "key_1", "key_2"
   *
   * @param type_name The fully qualified C++ type name of the transmitter component
   * @param name Name of the transmitter component
   * @param arg_list Arguments for the transmitter component
   * @param omit_term Boolean flag controlling whether or not a default downstream receptive
   *                  scheduling term is added. If true, no scheduling term is added.
   * @return Handle<Transmitter> Handle to newly created transmitter component
   */
  Handle<Transmitter> addTransmitter(const char* type_name, const char* name = nullptr,
                                     const std::vector<Arg>& arg_list = {}, bool omit_term = false);

  /**
   * @brief Transmitter component lookup using name
   *
   * @param name name of a transmitter component which has been previously created
   * @return Handle<Transmitter> Handle to transmitter component if found, Null handle if
   * no such component.
   */
  Handle<Transmitter> getTransmitter(const char* name) {
    for (auto it = tx_queues_.begin(); it != tx_queues_.end(); ++it) {
      if (!strcmp(it->second->name(), name)) { return it->second; }
    }
    return Handle<Transmitter>::Null();
  }

  /**
   * @brief Adds a component of Receiver type T with name and sets the parameter values from Args
   *
   * Name of the receiver should match the parameter name of the underlying codelet
   * The name of the receiver component is updated based on the parameter rank info.
   * A Message available scheduling term is also added to monitor the receiver component
   *
   * If codelet parameter is a scalar, name of the receiver is also same as the parameter key
   * Parameter<Handle<Receiver>> | name - "key"
   *
   * If codelet parameter is a vector/array, the name of the receiver component is key_%d where
   * 'd' is the index of this receiver in the codelet parameter.
   * Parameter<Vector<Handle<Receiver>> | name - "key_0", "key_1", "key_2"
   *
   * @tparam T Type of Receiver. Must be derived from nvidia::gxf::Receiver
   * @param name  Name of the receiver component
   * @param omit_term Boolean flag controlling whether or not a default message available
   *                  scheduling term is added. If true, no scheduling term is added.
   * @param args Args must be of type Arg
   * @return Handle<T> Handle to newly created receiver component
   */
  template <typename T, typename... Args>
  Handle<T> addReceiver(const char* name, bool omit_term = false, Args... args) {
    static_assert(std::is_base_of<Receiver, T>::value, "Requested type is not a Receiver");
    auto maybe_rx_name = formatRxName(name);
    if (!maybe_rx_name) { return Handle<T>::Null(); }
    auto rx_name = maybe_rx_name.value();

    auto rx = add<T>(rx_name.c_str(), args...);
    if (!rx) { return rx; }

    if (!omit_term) {
      // auto adds a message available scheduling term
      auto term = this->add<MessageAvailableSchedulingTerm>(rx_name.c_str());
      term->setReceiver(rx);
    }

    std::string full_name = std::string(this->name()) + "/" + rx_name;
    auto result = updatePort(name, full_name);
    if (!result) {
      GXF_LOG_ERROR("Failed to add Receiver [%s] with error [%s]", rx_name.c_str(),
                    GxfResultStr(result.error()));
    }

    return rx;
  }

  /**
   * @brief Adds a component of Receiver of the corresponding type_name.
   * Name of the receiver should match the parameter name of the underlying codelet
   * The name of the receiver component is updated based on the parameter rank info.
   *
   * If codelet parameter is a scalar, name of the receiver is also same as the parameter key
   * Parameter<Handle<Receiver>> | name - "key"
   *
   * If codelet parameter is a vector/array, the name of the receiver component is key_%d where
   * 'd' is the index of this receiver in the codelet parameter.
   * Parameter<Vector<Handle<Receiver>> | name - "key_0", "key_1", "key_2"
   *
   * @param type_name The fully qualified C++ type name of the receiver component
   * @param name Name of the receiver component
   * @param arg_list Arguments for the receiver component
   * @param omit_term Boolean flag controlling whether or not a default message available
   *                  scheduling term is added. If true, no scheduling term is added.
   * @return Handle<Receiver> Handle to newly created receiver component
   */
  Handle<Receiver> addReceiver(const char* type_name, const char* name = nullptr,
                               const std::vector<Arg>& arg_list = {}, bool omit_term = false);

  /**
   * @brief Receiver component lookup using name
   *
   * @param name name of a receiver component which has been previously created
   * @return Handle<Receiver> Handle to receiver component if found, Null handle if
   * no such component.
   */
  Handle<Receiver> getReceiver(const char* name) {
    for (auto it = rx_queues_.begin(); it != rx_queues_.end(); ++it) {
      if (!strcmp(it->second->name(), name)) { return it->second; }
    }
    return Handle<Receiver>::Null();
  }

  /**
   * @brief Update the capacity and min_size parameter of a transmitter and its corresponding
   * downstream receptive scheduling term
   *
   * @param name Name of the transmitter component
   * @param capacity capacity of the transmitter to be set
   * @param policy policy of the transmitter to be set
   * @param min_size min size of the downstream receptive term to be set
   * @return Expected<void> On success the function returns Success
   */
  Expected<void> configTransmitter(const char* name, uint64_t capacity, uint64_t policy,
                                   uint64_t min_size) {
    if (tx_queues_.find(name) == tx_queues_.end()) {
      GXF_LOG_ERROR("Transmitter [%s] is not found in entity [%s]", name, this->name());
      return Unexpected{GXF_ENTITY_COMPONENT_NOT_FOUND};
    }

    Handle<Component> tx = tx_queues_.at(name);
    tx->setParameter("capacity", capacity);
    tx->setParameter("policy", policy);

    if (terms_.find(name) == terms_.end()) {
      GXF_LOG_ERROR("Scheduling term [%s] is not found in entity [%s]", name, this->name());
      return Unexpected{GXF_ENTITY_COMPONENT_NOT_FOUND};
    }

    Handle<Component> term = terms_.at(name);
    term->setParameter("min_size", min_size);

    return Success;
  }

  /**
   * @brief Update the capacity and min_size parameter of a receiver and its corresponding
   * message available scheduling term
   *
   * @param name Name of the receiver component
   * @param capacity capacity of the receiver to be set
   * @param policy policy of the receiver to be set
   * @param min_size min size of the message available term to be set
   * @return Expected<void> On success the function returns Success
   */
  Expected<void> configReceiver(const char* name, uint64_t capacity, uint64_t policy,
                                uint64_t min_size) {
    if (rx_queues_.find(name) == rx_queues_.end()) {
      GXF_LOG_ERROR("Receiver [%s] is not found in entity [%s]", name, this->name());
      return Unexpected{GXF_ENTITY_COMPONENT_NOT_FOUND};
    }

    Handle<Component> rx = rx_queues_.at(name);
    rx->setParameter("capacity", capacity);
    rx->setParameter("policy", policy);

    if (terms_.find(name) == terms_.end()) {
      GXF_LOG_ERROR("Scheduling term [%s] is not found in entity [%s]", name, this->name());
      return Unexpected{GXF_ENTITY_COMPONENT_NOT_FOUND};
    }

    Handle<Component> term = terms_.at(name);
    term->setParameter("min_size", min_size);

    return Success;
  }

  Expected<void> activate() { return entity_.activate(); }

  Expected<void> deactivate() { return entity_.deactivate(); }

  gxf_context_t context() const { return entity_.context(); }

  gxf_uid_t eid() const { return entity_.eid(); }

  bool is_null() const { return entity_.is_null(); }

  Handle<Codelet> get_codelet() { return codelet_; }

  /**
   * @brief The name of the entity or empty string if no name has been given to the entity.
   *
   * @return const char* pointer to name of the entity
   */
  const char* name() const { return entity_.name(); }

  /**
   * @brief Given a name for a transmitter to be connected to the codelet, return a formatted
   * string back which can be used for a new transmitter component creation
   * If the codelet's tx parameter is a scalar, the tx name is the same as the parameter key
   * If the codelet's tx parameter is a vector, the tx name would be "key_0", "key_1" ...
   *
   * @param tx_name name of transmitter component
   * @return Expected<std::string> formatted name of transmitter component
   */
  Expected<std::string> formatTxName(const char* tx_name);

  /**
   * @brief Given a name for a receiver to be connected to the codelet, return a formatted
   * string back which can be used for a new receiver component creation
   * If the codelet's rx parameter is a scalar, the rx name is the same as the parameter key
   * If the codelet's rx parameter is a vector, the rx name would be "key_0", "key_1" ...
   *
   * @param rx_name name of receiver component
   * @return Expected<std::string> formatted name of receiver component
   */
  Expected<std::string> formatRxName(const char* rx_name);

  Expected<void> updatePort(const char* key, std::string value);

 private:
  // Creates a generic component of type T and sets the parameter values from arg_list.
  //
  // Transmitters, Receivers, Clocks and Scheduling Term component names have to be unique.
  // If the component type has been registered on any of the extensions which have been loaded, the
  // new type would be registered to a runtime extension.
  template <typename T, typename... Args>
  Expected<Handle<T>> addComponent(const char* name = nullptr,
                                   const std::vector<Arg>& arg_list = {}) {
    static_assert(std::is_base_of<Component, T>::value, "Requested type is not a component");
    // Check if another component with same name and base type exists
    // return failure if it exists, otherwise proceed with component creation
    if constexpr (std::is_base_of<Codelet, T>::value) {
      if (!codelet_.is_null() && !strcmp(codelet_->name(), name)) {
        GXF_LOG_ERROR("Codelet with same name [%s] already exists in entity [%s]", name,
                      entity_.name());
        return Unexpected{GXF_FAILURE};
      }
    } else if constexpr (std::is_base_of<SchedulingTerm, T>::value) {
      if (terms_.find(name) != terms_.end()) {
        GXF_LOG_ERROR("Scheduling term with same name [%s] already exists in entity [%s]", name,
                      entity_.name());
        return Unexpected{GXF_FAILURE};
      }
    } else if constexpr (std::is_base_of<Transmitter, T>::value) {
      if (tx_queues_.find(name) != tx_queues_.end()) {
        GXF_LOG_ERROR("Transmitter with same name [%s] already exists in entity [%s]", name,
                      entity_.name());
        return Unexpected{GXF_FAILURE};
      }
    } else if constexpr (std::is_base_of<Receiver, T>::value) {
      if (rx_queues_.find(name) != rx_queues_.end()) {
        GXF_LOG_ERROR("Receiver with same name [%s] already exists in entity [%s]", name,
                      entity_.name());
        return Unexpected{GXF_FAILURE};
      }
    } else if constexpr (std::is_base_of<Clock, T>::value) {
      if (clocks_.find(name) != clocks_.end()) {
        GXF_LOG_ERROR("Clock with same name [%s] already exists in entity [%s]", name,
                      entity_.name());
        return Unexpected{GXF_FAILURE};
      }
    }

    Expected<Handle<T>> maybe_component = entity_.add<T>(name);
    if (!maybe_component) {
      GXF_LOG_ERROR("Failed to add handle for component named [%s] to entity [%s] with error %s",
                    name, entity_.name(), GxfResultStr(maybe_component.error()));
      return ForwardError(maybe_component);
    }
    Handle<T> component = maybe_component.value();

    for (auto arg : arg_list) { applyArg(component, arg); }

    if constexpr (std::is_base_of<Codelet, T>::value) {
      if (!codelet_.is_null()) {
        GXF_LOG_DEBUG(
            "Graph Entity is already configured with a codelet [%s]. New codelet "
            "will be created but not managed by entity [%s]",
            codelet_->name(), entity_.name());
      } else {
        codelet_ = component;
      }
    } else if constexpr (std::is_base_of<SchedulingTerm, T>::value) {
      terms_.emplace(name, component);
    } else if constexpr (std::is_base_of<Transmitter, T>::value) {
      tx_queues_.emplace(name, component);
    } else if constexpr (std::is_base_of<Receiver, T>::value) {
      rx_queues_.emplace(name, component);
    } else if constexpr (std::is_base_of<Clock, T>::value) {
      clocks_.emplace(name, component);
    }

    components_.emplace(component.get()->cid(), component);
    return component;
  }

  // Create a Handle of the specified type corresponding to a fully qualified C++ type name and
  // add it to the entity.
  template <typename T>
  Expected<Handle<T>> createHandle(const char* type_name, const char* name = nullptr) {
    gxf_tid_t tid;
    gxf_context_t context = entity_.context();

    gxf_result_t result = GxfComponentTypeId(context, type_name, &tid);
    if (!isSuccessful(result)) {
      GXF_LOG_ERROR("Typename [%s] not found. Is this type registered?", type_name);
      return Unexpected{result};
    }

    auto maybe_untyped_handle = entity_.add(tid, name);
    if (!maybe_untyped_handle) {
      GXF_LOG_ERROR("Failed to add handle for [%s] to entity [%s] with error %s", type_name,
                    entity_.name(), GxfResultStr(maybe_untyped_handle.error()));
      return ForwardError(maybe_untyped_handle);
    }

    auto maybe_handle = Handle<T>::Create(maybe_untyped_handle.value());
    if (!maybe_handle) {
      GXF_LOG_ERROR("Failed to convert untyped handle to handle with error %s",
                    GxfResultStr(maybe_handle.error()));
      return ForwardError(maybe_handle);
    }

    return maybe_handle.value();
  }

  // A single codelet per GraphEntity
  Handle<Codelet> codelet_{Handle<Codelet>::Null()};

  // global component lookup based on uid
  std::map<gxf_uid_t, Handle<Component>> components_;

  // scheduling term component lookup by name
  std::map<std::string, Handle<SchedulingTerm>> terms_;

  // transmitter component lookup by name
  std::map<std::string, Handle<Transmitter>> tx_queues_;

  // receiver component lookup by name
  std::map<std::string, Handle<Receiver>> rx_queues_;

  // clock component lookup by name
  std::map<std::string, Handle<Clock>> clocks_;

  // resource lookup by name
  std::map<std::string, Handle<ResourceBase>> resources_;

  gxf_uid_t eid_{kNullUid};
  Entity entity_;
};

typedef std::shared_ptr<GraphEntity> GraphEntityPtr;

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_GRAPH_ENTITY_HPP_
