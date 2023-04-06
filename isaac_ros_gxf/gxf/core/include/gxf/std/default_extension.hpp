/*
 * SPDX-FileCopyrightText: Copyright (c) 2021-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#ifndef NVIDIA_GXF_STD_DEFAULT_EXTENSION_HPP_
#define NVIDIA_GXF_STD_DEFAULT_EXTENSION_HPP_

#include <algorithm>
#include <memory>
#include <string>
#include <type_traits>

#include "common/fixed_vector.hpp"
#include "common/type_name.hpp"
#include "gxf/core/expected.hpp"
#include "gxf/std/codelet.hpp"
#include "gxf/std/component_allocator.hpp"
#include "gxf/std/extension.hpp"
#include "gxf/std/new_component_allocator.hpp"

namespace nvidia {
namespace gxf {

namespace detail {

struct VoidBaseHelper {};

template <typename T>
struct BaseTypenameAsString {
  static const char* Value() { return TypenameAsString<T>(); }
};

template <>
struct BaseTypenameAsString<VoidBaseHelper> {
  static constexpr const char* Value() { return ""; }
};

}  // namespace detail

// A standard component factor for GXF extensions. It keeps track of all components in the
// extensions and provides mechanisms to create components.
class DefaultExtension : public Extension {
 public:
  ~DefaultExtension() override = default;

  // Sets the extension metadata info
  gxf_result_t setInfo_abi(gxf_tid_t tid, const char* name, const char* desc, const char* author,
                           const char* version, const char* license) override;


  // Capture extension display metadata
  gxf_result_t setDisplayInfo_abi(const char* display_name, const char* category,
                                  const char* brief) override;

  // Gets description of the extension and list of components it provides
  gxf_result_t getInfo_abi(gxf_extension_info_t* info) override;

  gxf_result_t checkInfo_abi() override;

  // Gets description of specified component (No parameter information)
  gxf_result_t getComponentInfo_abi(const gxf_tid_t tid, gxf_component_info_t* info) override;

  gxf_result_t registerComponents_abi(gxf_context_t context) override;

  gxf_result_t getComponentTypes_abi(gxf_tid_t* pointer, size_t* size) override;

  gxf_result_t allocate_abi(gxf_tid_t tid, void** out_pointer) override;

  gxf_result_t deallocate_abi(gxf_tid_t tid, void* pointer) override;

  gxf_result_t getParameterInfo_abi(gxf_context_t context, const gxf_tid_t cid, const char* key,
               gxf_parameter_info_t* info) override;

  template <typename T, typename Base = detail::VoidBaseHelper>
  Expected<void> add(gxf_tid_t tid, const char* description, const char* display_name = "",
                     const char* brief = "") {
    static_assert(std::is_same<Base, detail::VoidBaseHelper>::value
                  || std::is_base_of<Base, T>::value,
                  "The given base class is not actually a base class");
    static_assert(!std::is_base_of<Codelet, T>::value ||
                  (std::is_base_of<Codelet, Base>::value || std::is_same<Codelet, T>::value),
                  "If a component derives from Codelet then its base class also needs to derive "
                  "from Codelet. Have you used Component as base class instead of Codelet?");

    if (find(tid)) {
      return Unexpected{GXF_FACTORY_DUPLICATE_TID};
    }

    std::string display_name_str{display_name};
    if (display_name_str.length() > 50) {
      GXF_LOG_ERROR("Component display name '%s' exceeds 50 characters", display_name);
      return Unexpected{GXF_ARGUMENT_OUT_OF_RANGE};
    }

    std::string brief_str{brief};
    if (brief_str.length() > 128) {
      GXF_LOG_ERROR("Component brief '%s' exceeds 128 characters", brief);
      return Unexpected{GXF_ARGUMENT_OUT_OF_RANGE};
    }

    std::string description_str{description};
    if (description_str.length() > 1026) {
      GXF_LOG_ERROR("Component description '%s' exceeds 1026 characters", description);
      return Unexpected{GXF_ARGUMENT_OUT_OF_RANGE};
    }

    auto result = entries_.push_back({
        tid, TypenameAsString<T>(), detail::BaseTypenameAsString<Base>::Value(),
        description_str, display_name_str, brief_str,
        std::is_abstract<T>::value ? nullptr : std::make_unique<NewComponentAllocator<T>>()});
    if (!result) {
      GXF_LOG_WARNING("Exceeding maximum number of components");
      return Unexpected{GXF_EXCEEDING_PREALLOCATED_SIZE};
    }
    return Success;
  }

 private:
  struct Entry {
    gxf_tid_t tid;
    std::string name;
    std::string base;
    std::string description;
    std::string display_name;
    std::string brief;
    std::unique_ptr<ComponentAllocator> allocator;
  };

  Expected<Entry&> find(const gxf_tid_t& tid);

  FixedVector<Entry, kMaxComponents> entries_;
  gxf_tid_t tid_{GxfTidNull()};
  std::string name_;
  std::string description_;
  std::string author_;
  std::string extension_version_;
  std::string gxf_core_version_{kGxfCoreVersion};
  std::string license_;
  std::string display_name_;
  std::string category_;
  std::string brief_;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_STD_DEFAULT_EXTENSION_HPP_
