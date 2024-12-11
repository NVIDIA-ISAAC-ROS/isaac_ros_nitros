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

#ifndef NVIDIA_GXF_ENTITY_GROUP_HPP_
#define NVIDIA_GXF_ENTITY_GROUP_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "gxf/app/graph_entity.hpp"

namespace nvidia {
namespace gxf {

/**
 * @brief  A wrapper over EntityGroup C APIs to manage a programmable
 * entity group in C++ API layer
 *
 */
class EntityGroup {
 public:
  EntityGroup() = default;

  ~EntityGroup() = default;

  EntityGroup(const EntityGroup&) = delete;

  EntityGroup& operator=(const EntityGroup&) = delete;

  /**
   * @brief Creates a programmable entity group with the runtime context and sets its name
   *
   * @param context A valid GXF context
   * @param name Name of the graph entity
   */
  Expected<void> setup(gxf_context_t context, const char* name);

  /**
   * @brief Add single entity into this entity group.
   * All Resouce components within the group will be automatically resolved to owner components
   * that has corresponding type of Resource members registered.
   *
   * @param GraphEntityPtr An entity in C++ API representation
   */
  Expected<void> add(GraphEntityPtr entity);

  /**
   * @brief Add a list of entities into this entity group.
   * All Resouce components within the group will be automatically resolved to owner components
   * that has corresponding type of Resource members registered.
   *
   * @param entity_members A list of entities in C++ API representation
   */
  Expected<void> add(std::vector<GraphEntityPtr> entity_members);

  /**
   * @return This entity group ID.
   */
  gxf_uid_t gid() const { return gid_; }

  /**
   * @return This entity group name.
   */
  std::string name() const { return name_; }

 private:
  gxf_uid_t gid_{kNullUid};
  std::string name_;
  std::map<std::string, GraphEntityPtr> entity_members_;
};

typedef std::shared_ptr<EntityGroup> EntityGroupPtr;

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_ENTITY_GROUP_HPP_
