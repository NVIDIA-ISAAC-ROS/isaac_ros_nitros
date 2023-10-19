// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#ifndef NVIDIA_GXF_GXF_STD_RESOURCE_MANAGER_HPP_
#define NVIDIA_GXF_GXF_STD_RESOURCE_MANAGER_HPP_

#include <utility>
#include <vector>

#include "gxf/core/expected.hpp"
#include "gxf/core/gxf.h"
#include "gxf/core/handle.hpp"

namespace nvidia {
namespace gxf {

/// @brief Provide lookup from eid & resource type to resource_cids
class ResourceManager {
 public:
  ResourceManager(gxf_context_t context);
  virtual ~ResourceManager() = default;

  // Static public interface
  static Expected<gxf_uid_t> findEntityResourceByTypeName(gxf_context_t context,
                               gxf_uid_t eid, const char* type_name,
                               const char* target_resource_name = nullptr);

  // Static public interface
  static Expected<gxf_uid_t> findComponentResourceByTypeName(gxf_context_t context,
                               gxf_uid_t cid, const char* type_name,
                               const char* target_resource_name = nullptr);


  // Static public interface
  // Find Resource of type Handle<T> belong to entity eid
  template <typename T>
  static Expected<Handle<T>> findEntityResource(gxf_context_t context, gxf_uid_t eid,
                                            const char* target_resource_name = nullptr) {
    const char* type_name = TypenameAsString<T>();
    Expected<gxf_uid_t> maybe_cid = ResourceManager::findEntityResourceByTypeName(context,
                                                       eid, type_name, target_resource_name);
    if (!maybe_cid) { return ForwardError(maybe_cid); }
    return Handle<T>::Create(context, maybe_cid.value());
  }


  // Find Resource of type Handle<T> belong to entity eid
  template <typename T>
  Expected<Handle<T>> findEntityResource(gxf_uid_t eid,
                                         const char* target_resource_name = nullptr) {
    // object member function calls class static function
    return ResourceManager::findEntityResource<T>(context_, eid, target_resource_name);
  }


  // Find Resource of type Handle<T> belong to component cid
  template <typename T>
  Expected<Handle<T>> findComponentResource(gxf_uid_t cid,
                                            const char* target_resource_name = nullptr) {
    const char* type_name = TypenameAsString<T>();
    Expected<gxf_uid_t> maybe_cid = ResourceManager::findComponentResourceByTypeName(context_,
                                                       cid, type_name, target_resource_name);
    if (!maybe_cid) { return ForwardError(maybe_cid); }
    return Handle<T>::Create(context_, maybe_cid.value());
  }

 private:
  gxf_context_t context_;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_GXF_STD_RESOURCE_MANAGER_HPP_
