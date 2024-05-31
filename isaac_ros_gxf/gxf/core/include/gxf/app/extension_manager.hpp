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

#ifndef NVIDIA_GXF_GRAPH_EXTENSION_MANAGER_HPP_
#define NVIDIA_GXF_GRAPH_EXTENSION_MANAGER_HPP_

#include <map>
#include <set>
#include <shared_mutex>  // NOLINT
#include <vector>

#include "common/fixed_vector.hpp"
#include "gxf/core/expected.hpp"
#include "gxf/std/extension.hpp"

namespace nvidia {
namespace gxf {

/**
 * @brief Loads extensions and allows to create instances of their components.
 *
 */
class ExtensionManager {
 public:
  ExtensionManager() = default;
  ~ExtensionManager() = default;

  ExtensionManager(const ExtensionManager&) = delete;
  ExtensionManager(ExtensionManager&&) = delete;
  ExtensionManager& operator=(const ExtensionManager&) = delete;
  ExtensionManager& operator=(ExtensionManager&&) = delete;

  // Loads a GXF extension from the given file
  Expected<void> registerExtensions(gxf_context_t context);
  Expected<void> load(const char* filename);
  Expected<void> load(Extension* extension, void* handle = nullptr);
  Expected<void> loadManifest(const char* filename);
  Expected<void> unloadAll();

  // Gets list of TIDs for loaded Extensions.
  // [in/out] extension_num is for capacity of parameter extensions
  // [out] extensions is memory to write TIDs to
  Expected<void> getExtensions(uint64_t* extension_count, gxf_tid_t* extensions);

  // Gets description for specified (loaded) extension and list of components It provides
  Expected<void> getExtensionInfo(gxf_tid_t eid, gxf_extension_info_t* info);

 private:
  std::set<void*> handles_;
  std::map<gxf_tid_t, Extension*> component_factory_;
  std::map<gxf_tid_t, Extension*> extension_factory_;
  std::vector<Extension*> factories_;

  mutable std::shared_timed_mutex mutex_;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_GRAPH_EXTENSION_MANAGER_HPP_
