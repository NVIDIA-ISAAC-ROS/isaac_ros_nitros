/*
 * SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#ifndef NVIDIA_GXF_STD_PARAMETER_REGISTRAR_HPP_
#define NVIDIA_GXF_STD_PARAMETER_REGISTRAR_HPP_

#include <map>
#include <memory>
#include <shared_mutex>  // NOLINT
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "common/memory_utils.hpp"
#include "common/type_utils.hpp"
#include "gxf/core/gxf.h"
#include "gxf/core/handle.hpp"
#include "gxf/core/parameter.hpp"
#include "gxf/std/type_registry.hpp"

namespace nvidia {
namespace gxf {

// @brief Struct specifying parameters for registering parameter
template <typename T>
struct ParameterInfo {
  // Key used to access the parameter. Required.
  const char* key = nullptr;
  // Brief description. Optional.
  const char* headline = nullptr;
  // Detailed description. Optional.
  const char* description = nullptr;
  // Applicable platform list separated by comma. Optional.
  const char* platform_information = nullptr;

  // Default value if not provided otherwise. Not applicable to Handle parameter. Optional.
  Expected<T> value_default = Unexpected{GXF_PARAMETER_NOT_INITIALIZED};
  // Minimal value, Maximum value and Minimal step for arithmetic types. Optional.
  Expected<std::array<T, 3>> value_range = Unexpected{GXF_PARAMETER_NOT_INITIALIZED};

  // Bit flags about properties like if it is required etc.
  gxf_parameter_flags_t flags = GXF_PARAMETER_FLAGS_NONE;

  // Max allowed is rank 8
  static constexpr const int32_t kMaxRank = 8;
  // Parameters can be tensors and this value indicates its rank. For a "scalar" rank is 0.
  int32_t rank = 0;
  // Parameters can be tensors and this is its shape. Only values up to rank must be set. If rank
  // is 0 no value must be set. -1 can be used to indicate a dynamic size which is not fixed at
  // compile time.
  int32_t shape[kMaxRank] = {1};
};

class Component;

template <typename T>
struct ParameterTypeTrait {
  static constexpr const char* type_name = "(custom)";
  static constexpr const gxf_parameter_type_t type = GXF_PARAMETER_TYPE_CUSTOM;
  static constexpr const bool is_arithmetic = false;
};

template <typename T>
struct ParameterTypeTrait<Handle<T>> {
  static constexpr const char* type_name = "(handle)";
  static constexpr const gxf_parameter_type_t type = GXF_PARAMETER_TYPE_HANDLE;
  static constexpr const bool is_arithmetic = false;
};

template <>
struct ParameterTypeTrait<int8_t> {
  static constexpr const char* type_name = "Int8";
  static constexpr const gxf_parameter_type_t type = GXF_PARAMETER_TYPE_INT8;
  static constexpr const bool is_arithmetic = true;
};

template <>
struct ParameterTypeTrait<int16_t> {
  static constexpr const char* type_name = "Int16";
  static constexpr const gxf_parameter_type_t type = GXF_PARAMETER_TYPE_INT16;
  static constexpr const bool is_arithmetic = true;
};

template <>
struct ParameterTypeTrait<int32_t> {
  static constexpr const char* type_name = "Int32";
  static constexpr const gxf_parameter_type_t type = GXF_PARAMETER_TYPE_INT32;
  static constexpr const bool is_arithmetic = true;
};

template <>
struct ParameterTypeTrait<int64_t> {
  static constexpr const char* type_name = "Int64";
  static constexpr const gxf_parameter_type_t type = GXF_PARAMETER_TYPE_INT64;
  static constexpr const bool is_arithmetic = true;
};

template <>
struct ParameterTypeTrait<uint8_t> {
  static constexpr const char* type_name = "UInt8";
  static constexpr const gxf_parameter_type_t type = GXF_PARAMETER_TYPE_UINT8;
  static constexpr const bool is_arithmetic = true;
};

template <>
struct ParameterTypeTrait<uint16_t> {
  static constexpr const char* type_name = "UInt16";
  static constexpr const gxf_parameter_type_t type = GXF_PARAMETER_TYPE_UINT16;
  static constexpr const bool is_arithmetic = true;
};

template <>
struct ParameterTypeTrait<uint32_t> {
  static constexpr const char* type_name = "UInt32";
  static constexpr const gxf_parameter_type_t type = GXF_PARAMETER_TYPE_UINT32;
  static constexpr const bool is_arithmetic = true;
};

template <>
struct ParameterTypeTrait<uint64_t> {
  static constexpr const char* type_name = "UInt64";
  static constexpr const gxf_parameter_type_t type = GXF_PARAMETER_TYPE_UINT64;
  static constexpr const bool is_arithmetic = true;
};

template <>
struct ParameterTypeTrait<float> {
  static constexpr const char* type_name = "Float32";
  static constexpr const gxf_parameter_type_t type = GXF_PARAMETER_TYPE_FLOAT32;
  static constexpr const bool is_arithmetic = true;
};

template <>
struct ParameterTypeTrait<double> {
  static constexpr const char* type_name = "Float64";
  static constexpr const gxf_parameter_type_t type = GXF_PARAMETER_TYPE_FLOAT64;
  static constexpr const bool is_arithmetic = true;
};

template <>
struct ParameterTypeTrait<char*> {
  static constexpr const char* type_name = "String";
  static constexpr const gxf_parameter_type_t type = GXF_PARAMETER_TYPE_STRING;
  static constexpr const bool is_arithmetic = false;
};

template <>
struct ParameterTypeTrait<std::string> {
  static constexpr const char* type_name = "String";
  static constexpr const gxf_parameter_type_t type = GXF_PARAMETER_TYPE_STRING;
  static constexpr const bool is_arithmetic = false;
};

template <>
struct ParameterTypeTrait<bool> {
  static constexpr const char* type_name = "Boolean";
  static constexpr const gxf_parameter_type_t type = GXF_PARAMETER_TYPE_BOOL;
  static constexpr const bool is_arithmetic = false;
};

template <>
struct ParameterTypeTrait<FilePath> {
  static constexpr const char* type_name = "File";
  static constexpr const gxf_parameter_type_t type = GXF_PARAMETER_TYPE_FILE;
  static constexpr const bool is_arithmetic = false;
};

// Stores parameters for an GXF application
class ParameterRegistrar {
 public:
  // Container for a single object of any type
  // Contained object is dynamically allocated on the heap and properly destructed when finished
  class TypeEraser {
   public:
    TypeEraser() = default;
    TypeEraser(const TypeEraser& other) { *this = other; }
    TypeEraser(TypeEraser&& other) { *this = std::move(other); }
    template <typename T, EnableIf_t<!IsSame_v<Decay_t<T>, TypeEraser>, TypeEraser*> = nullptr>
    TypeEraser(T&& value) { *this = std::forward<T>(value); }
    ~TypeEraser() { reset(); }

    TypeEraser& operator=(const TypeEraser& other) {
      if (other.instance_) {
        instance_ = other.instance_->clone();
      }
      return *this;
    }

    TypeEraser& operator=(TypeEraser&& other) {
      instance_ = std::move(other.instance_);
      return *this;
    }

    template <typename T>
    EnableIf_t<!IsSame_v<Decay_t<T>, TypeEraser>, TypeEraser&> operator=(T&& value) {
      instance_ = MakeUniqueNoThrow<storage_impl<Decay_t<T>>>(std::forward<T>(value));
      return *this;
    }

    bool has_value() const { return static_cast<bool>(instance_); }
    const void* get() const { return has_value() ? instance_->get() : nullptr; }
    void reset() { instance_.reset(); }

   private:
    struct storage_base {
      virtual ~storage_base() = default;
      virtual const void* get() const = 0;
      virtual std::unique_ptr<storage_base> clone() const = 0;
    };

    template <typename T>
    struct storage_impl final : public storage_base {
      template <typename... Args>
      storage_impl(Args&&... args) : value(std::forward<Args>(args)...) {}

      const void* get() const override { return &value; }

      std::unique_ptr<storage_base> clone() const override {
        return MakeUniqueNoThrow<storage_impl<T>>(value);
      }

      T value;
    };

    std::unique_ptr<storage_base> instance_;
  };

  // Struct to hold information about a single parameter
  struct ComponentParameterInfo {
    std::string key;
    std::string headline;
    std::string description;

    std::string platform_information;

    gxf_parameter_type_t type;
    gxf_tid_t handle_tid = GxfTidNull();
    bool is_arithmetic;

    gxf_parameter_flags_t flags;

    // params to be accessed only via getDefaultValue(...)
    // and getNumericRange(...) apis
    TypeEraser default_value;
    // Range info as [min, max, step]
    std::array<TypeEraser, 3> value_range;

    int32_t rank = 0;
    int32_t shape[ParameterInfo<int32_t>::kMaxRank];
  };

  // Struct to hold information about all the parameters in a component
  struct ComponentInfo {
    std::string type_name;
    std::vector<std::string> parameter_keys;
    std::unordered_map<std::string, ComponentParameterInfo> parameters;
  };

  ParameterRegistrar() = default;

  template <typename T>
  Expected<void> registerComponentParameter(gxf_tid_t tid, const std::string& type_name,
                                            const ParameterInfo<T>& parameter_info);

  // Add's a parameterless component to parameter registrar. These are useful to lookup component
  // tid using component type name when parameters of type Handle<T> are being registered
  void addParameterlessType(const gxf_tid_t tid, std::string type_name);

  // Check if parameter registrar has a component
  bool hasComponent(const gxf_tid_t tid) const;

  // Get the number of parameters in a component
  size_t componentParameterCount(const gxf_tid_t tid) const;

  // Get the list of parameter keys in a component
  Expected<void> getParameterKeys(const gxf_tid_t tid, const char** keys, size_t& count) const;

  // Check if a component has a parameter
  Expected<bool> componentHasParameter(const gxf_tid_t tid, const char* key) const;

  // Get the default value of a component
  // Dependening on the data type of the parameter, default value has to be
  // casted into the data type specifed in gxf_parameter_type_t in gxf.h
  Expected<const void*> getDefaultValue(const gxf_tid_t tid, const char* key) const;

  // Load the numeric range of a parameter into gxf_parameter_info_t
  // Dependening on the data type of the parameter, numeric ranges value have to be
  // casted into the data type specifed in gxf_parameter_type_t in gxf.h
  Expected<bool> getNumericRange(const gxf_tid_t tid, const char* key,
                                 gxf_parameter_info_t* info) const;

  // fills the gxf_parameter_info_t struct with the info stored in the parameter registrar
  // for that specific component and parameter key
  Expected<void> getParameterInfo(const gxf_tid_t tid, const char* key,
                                  gxf_parameter_info_t* info) const;

  // Returns the pointer to ComponentParameterInfo object in ComponentInfo
  Expected<ComponentParameterInfo*> getComponentParameterInfoPtr(const gxf_tid_t tid,
                                                                 const char* key) const;

  // Get the tid of a component which has already been registered with the type registry,
  // returns Unexpected if component not found
  Expected<gxf_tid_t> tidFromTypename(std::string type_name) {
    for (const auto& cparam : component_parameters) {
      if (cparam.second->type_name == type_name) {
        return cparam.first;
      }
    }
    GXF_LOG_ERROR("Component type not found %s", type_name.c_str());
    return Unexpected{GXF_ENTITY_COMPONENT_NOT_FOUND};
  }

 private:
  // Verifies the info of a new parameter being registered and fills the info
  // in the ComponentParameterInfo struct which is returned
  Expected<void> registerComponentParameterImpl(gxf_tid_t tid, const std::string& type_name,
                                                ComponentParameterInfo& info);

  // Maintains a map of {tid : ComponentInfo}
  // component type name in ComponentInfo is used to look up the corresponding tid when a parameter
  // of handle<T> is used for that component
  std::map<gxf_tid_t, std::unique_ptr<ComponentInfo>> component_parameters;
};

// loads numeric range info from ComponentParameterInfo to gxf_parameter_info_t
template <typename T>
static bool getNumericRangeImpl(ParameterRegistrar::ComponentParameterInfo* ptr,
                                gxf_parameter_info_t* info) {
  static_assert(ParameterTypeTrait<T>::is_arithmetic);

  if (!ptr || !info) {
    return false;
  }

  info->numeric_min = nullptr;
  info->numeric_max = nullptr;
  info->numeric_step = nullptr;

  if (std::get<0>(ptr->value_range).has_value()) {
    info->numeric_min = std::get<0>(ptr->value_range).get();
  }
  if (std::get<1>(ptr->value_range).has_value()) {
    info->numeric_max = std::get<1>(ptr->value_range).get();
  }
  if (std::get<2>(ptr->value_range).has_value()) {
    info->numeric_step = std::get<2>(ptr->value_range).get();
  }

  return true;
}

// Provides a customizable step which can be used during parameter registration to modify
// parameter info given to registry before it is written to the parameter info storage.
template <typename T>
struct ParameterInfoOverride {
  Expected<void> apply(ParameterRegistrar* /*registrar*/,
                       ParameterRegistrar::ComponentParameterInfo& info) {
    info.type = ParameterTypeTrait<T>::type;
    info.is_arithmetic = ParameterTypeTrait<T>::is_arithmetic;
    info.handle_tid = GxfTidNull();
    return Success;
  }
};

// Specialized ParameterInfoOverride for parameters of type handle<T>
template <typename T>
struct ParameterInfoOverride<Handle<T>> {
  Expected<void> apply(ParameterRegistrar* registrar,
                       ParameterRegistrar::ComponentParameterInfo& info) {
    info.type = GXF_PARAMETER_TYPE_HANDLE;
    info.is_arithmetic = false;
    auto handle = registrar->tidFromTypename(TypenameAsString<T>());
    if (!handle) { return ForwardError(handle); }
    info.handle_tid = handle.value();
    return Success;
  }
};

template <typename T>
Expected<void> ParameterRegistrar::registerComponentParameter(
    gxf_tid_t tid, const std::string& type_name, const ParameterInfo<T>& registrar_info) {

  ComponentParameterInfo info;

  if (registrar_info.key == nullptr) {
    return Unexpected{GXF_ARGUMENT_NULL};
  }
  info.key = std::string{registrar_info.key};

  if (registrar_info.headline == nullptr) {
    return Unexpected{GXF_ARGUMENT_NULL};
  }
  info.headline = std::string{registrar_info.headline};

  if (registrar_info.description == nullptr) {
    return Unexpected{GXF_ARGUMENT_NULL};
  }
  info.description = std::string{registrar_info.description};

  // Handles platform information if present
  if (registrar_info.platform_information != nullptr) {
    info.platform_information = std::string{registrar_info.platform_information};
  }

  if (registrar_info.value_default) {
    info.default_value = *registrar_info.value_default;
  } else {
    info.default_value.reset();
  }

  if (registrar_info.value_range) {
    std::get<0>(info.value_range) = std::get<0>(*registrar_info.value_range);
    std::get<1>(info.value_range) = std::get<1>(*registrar_info.value_range);
    std::get<2>(info.value_range) = std::get<2>(*registrar_info.value_range);
  } else {
    std::get<0>(info.value_range).reset();
    std::get<1>(info.value_range).reset();
    std::get<2>(info.value_range).reset();
  }

  info.flags = registrar_info.flags;

  // Set the rank and make sure the shape is correct.
  info.rank = registrar_info.rank;
  if (info.rank > ParameterInfo<T>::kMaxRank) {
    return Unexpected{GXF_ARGUMENT_OUT_OF_RANGE};
  }

  for (int32_t i = 0; i < info.rank; i++) {
    info.shape[i] = registrar_info.shape[i];
  }
  for (int32_t i = info.rank; i < ParameterInfo<T>::kMaxRank; i++) {
    info.shape[i] = 1;
  }

  // Apply custom overrides
  ParameterInfoOverride<T> custom_override;
  const auto result = custom_override.apply(this, info);
  if (!result) {
    GXF_LOG_ERROR("Parameter Override failed for Component \"%s\" and Parameter \"%s\"",
                  type_name.c_str(), registrar_info.key);
    return result;
  }

  // register the parameter
  return registerComponentParameterImpl(tid, type_name, info);
}

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_STD_PARAMETER_REGISTRAR_HPP_
