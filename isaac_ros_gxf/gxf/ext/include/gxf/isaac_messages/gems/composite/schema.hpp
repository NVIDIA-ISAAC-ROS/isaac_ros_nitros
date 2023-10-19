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
#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "engine/core/math/types.hpp"
#include "engine/core/optional.hpp"
#include "gems/composite/quantity.hpp"

namespace nvidia {
namespace isaac {
namespace composite {

// A schema contains information about the quantities which are stored in the composite
class Schema {
 public:
  // Constructs schema from given quanties
  static std::optional<Schema> Create(std::vector<Quantity>&& quantities, std::string hash = "");
  // Constructs schema from scalar quantities of identical measure
  static std::optional<Schema> Create(
      const std::vector<std::string>& entities, const Measure measure);

  // Similar to `Create` but will Assert if schema is invalid
  Schema(std::vector<Quantity>&& quantities, std::string hash = "");
  // Similar to `Create` but will Assert if schema is invalid
  Schema(const std::vector<std::string>& entities, const Measure measure);

  // Empty schema
  Schema() = default;

  // Equality operator
  bool operator==(const Schema& other) const {
    return element_count_ == other.element_count_ && hash_ == other.hash_ &&
           quantities_ == other.quantities_;
  }

  // The hash of the schame
  const std::string& getHash() const { return hash_; }

  // Returns a list of all quantities
  const std::vector<Quantity>& getQuantities() const { return quantities_; }

  // Total number of values required to store the schema
  int getElementCount() const { return element_count_; }

  // Gets the starting index of a quantity in the list ofvalues. Returns nullopt if quantity is
  // not in the schema.
  std::optional<int> findQuantityValueIndex(const Quantity& quantity) const;

 private:
  // Updates `element_count_`, `values_offset_` and `index_` variables based on quantities.
  bool createIndex();

  // The hash of the currently used schema
  std::string hash_;

  // All quantities which are part of this schema
  std::vector<Quantity> quantities_;

  // Total number of elements in the schema
  int element_count_ = 0;

  // Indies of first quantity element in the values vector
  std::unordered_map<Quantity, int, QuantityHash, QuantityEquality> index_;
};

}  // namespace composite
}  // namespace isaac
}  // namespace nvidia
