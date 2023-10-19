// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2021-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#pragma once

#include <mutex>

#include "common/unique_index_map.hpp"
#include "gems/composite/schema.hpp"
#include "gxf/core/component.hpp"

namespace nvidia {
namespace isaac {

// CompositeSchemaServer is a central repository for storing composite schemas across multiple
// components. When a Schema is loaded to the server, the server will then provide an 64 bit key
// that is guaranteed to be unique for the lifetime of the CompositeSchemaServer. This allows
// components to pass composite messages along with the key, rather than needing to encode the whole
// Schema in the message.
class CompositeSchemaServer : public gxf::Component {
 public:
  gxf_result_t registerInterface(gxf::Registrar* registrar) override;
  gxf_result_t initialize() override;

  // Stores a schema into the server and returns a unique ID tied to the schema.
  gxf::Expected<uint64_t> add(const composite::Schema& schema);

  // Retrieves a schema from the server using the provided unique ID.
  gxf::Expected<composite::Schema> get(uint64_t uid) const;

  // Returns true if the unique index map is at maximum capacity.
  bool full() const { return (schemas_.size() == schemas_.capacity()); }

 private:
  gxf::Parameter<uint64_t> maximum_capacity_;

  // Map of uint64_t to schemas that provides unique keys for every insert.
  UniqueIndexMap<composite::Schema> schemas_;

  // Protect from concurrent read/write access to the schemas_.
  mutable std::mutex mutex_;
};

}  // namespace isaac
}  // namespace nvidia
