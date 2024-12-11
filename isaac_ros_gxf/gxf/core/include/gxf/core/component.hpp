// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2021-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#ifndef NVIDIA_GXF_CORE_COMPONENT_HPP
#define NVIDIA_GXF_CORE_COMPONENT_HPP

#include <set>
#include <string>
#include <utility>
#include <vector>

#include "common/assert.hpp"
#include "gxf/core/entity.hpp"
#include "gxf/core/expected.hpp"
#include "gxf/core/gxf.h"
#include "gxf/core/handle.hpp"
#include "gxf/core/parameter.hpp"
#include "gxf/core/parameter_storage.hpp"
#include "gxf/core/registrar.hpp"

namespace nvidia {
namespace gxf {

// Components are parts of an entity and provide their functionality. The Component class is the
// base class of all GXF components.
class Component {
 public:
  virtual ~Component() = default;

  Component(const Component& component) = delete;
  Component(Component&& component) = delete;
  Component& operator=(const Component& component) = delete;
  Component& operator=(Component&& component) = delete;

  // Used to register all parameters of the components. Do not use this function for other purposes
  // as it might be called at anytime by the runtime.
  //
  // Example:
  //   class Foo : public Component {
  //    public:
  //     gxf_result_t registerInterface(Registrar* registrar) override {
  //       registrar->parameter(count_, "count", 1);
  //     }
  //     Parameter<int> count_;
  //   };
  virtual gxf_result_t registerInterface(Registrar* registrar) {
    registrar_ = registrar;
    parameter_storage_ = registrar_->getParameterStorage();
    registrar->registerParameterlessComponent();
    return GXF_SUCCESS;
  }

  // Use to start the lifetime of a component and should be used instead of the constructor.
  // Called after all components of an entity are created. The order in which components within
  // the same entity are initialized is undefined.
  virtual gxf_result_t initialize() { return GXF_SUCCESS; }

  // Use to end the lifetime of a component and should be used instead of the deconstructor.
  // The order in which components within the same entity are deinitialized is undefined.
  virtual gxf_result_t deinitialize() { return GXF_SUCCESS; }

  gxf_context_t context() const noexcept { return context_; }
  gxf_uid_t eid() const noexcept { return eid_; }
  gxf_uid_t cid() const noexcept { return cid_; }
  gxf_tid_t tid() const noexcept { return tid_; }
  const char* type_name() const noexcept { return typename_; }

  // The entity which owns this component
  Entity entity() const noexcept {
    // FIXME(v1) check that value exists
    return Entity::Shared(context(), eid()).value();
  }

  // Gets the name of the component
  const char* name() const noexcept {
    const char* result;
    const gxf_result_t code = GxfComponentName(context(), cid(), &result);
    return (code == GXF_SUCCESS) ? result : "";
  }

  // This function shall only be called by GXF and is used to setup the component.
  void internalSetup(gxf_context_t context, gxf_uid_t eid, gxf_uid_t cid, Registrar* registrar) {
    context_ = context;
    eid_ = eid;
    cid_ = cid;
    GxfComponentType(context_, cid_, &tid_);
    GxfComponentTypeName(context_, tid_, &typename_);
    if (registrar) {
      registrar_ = registrar;
      parameter_storage_ = registrar_->getParameterStorage();
      parameter_registrar_ = registrar_->getParameterRegistrar();
    }
  }

  // query value of component parameter "key"
  template<typename T>
  Expected<T> getParameter(const char* key) {
    return parameter_storage_->get<T>(cid_, key);
  }

  // set the parameter "key" and of type T with value
  template <typename T>
  Expected<void> setParameter(const char* key, T value) {
    if (!parameter_storage_) { return Unexpected{GXF_ARGUMENT_NULL}; }
    return parameter_storage_->set<T>(cid_, key, std::move(value));
  }

  // set a parameter "key" of handle type with value
  template <typename T>
  Expected<void> setParameter(const char* key, Handle<T>& value) {
    if (!parameter_storage_) { return Unexpected{GXF_ARGUMENT_NULL}; }
    return parameter_storage_->setHandle(cid_, key, value->cid());
  }

  // set the parameter "key" with the value in yaml node
  Expected<void> parseParameter(const char* key, const YAML::Node& node, std::string prefix = "") {
    return parameter_storage_->parse(cid_, key, node, prefix);
  }

  // wrap the current value of the parameter "key" in a yaml node
  Expected<YAML::Node> wrapParameter(const char* key) {
    return parameter_storage_->wrap(cid_, key);
  }

  // query all parameters in the component of type T and return their ComponentParameterInfo struct
  template<typename T>
  Expected<std::vector<ParameterRegistrar::ComponentParameterInfo>> getParametersOfType() {
    gxf_tid_t tid = GxfTidNull();
    auto result = GxfComponentType(context_, cid_, &tid);
    if (!isSuccessful(result)) {
      GXF_LOG_ERROR("Failed to find component type for cid [%ld]", cid_);
      return Unexpected{result};
    }

    return parameter_registrar_->getParametersOfType<T>(tid);
  }

  // query componentParameterInfo of parameter "key"
  Expected<ParameterRegistrar::ComponentParameterInfo> getParameterInfo(const char* key) {
    gxf_tid_t tid = GxfTidNull();
    auto result = GxfComponentType(context_, cid_, &tid);
    if (!isSuccessful(result)) {
      GXF_LOG_ERROR("Failed to find component type for cid [%ld]", cid_);
      return Unexpected{result};
    }

    ParameterRegistrar::ComponentParameterInfo info =
        *(GXF_UNWRAP_OR_RETURN(parameter_registrar_->getComponentParameterInfoPtr(tid, key)));
    return info;
  }

 protected:
  Component() = default;

  gxf_context_t context_ = kNullContext;
  gxf_uid_t eid_ = kNullUid;
  gxf_uid_t cid_ = kNullUid;
  gxf_tid_t tid_ = GxfTidNull();
  const char* typename_ = "UNKNOWN";
  Registrar* registrar_ = nullptr;
  ParameterRegistrar* parameter_registrar_ = nullptr;
  ParameterStorage* parameter_storage_ = nullptr;
};

}  // namespace gxf
}  // namespace nvidia

#endif
