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
#include <utility>

#include "engine/core/math/types.hpp"
#include "gems/composite/measure.hpp"

namespace nvidia {
namespace isaac {
namespace composite {

// A quantity describes meta data for values stored in a composite.
struct Quantity {
  // Helper function to create a scalar quantity
  static Quantity Scalar(std::string entity, Measure measure) {
    return Quantity{std::move(entity), measure, ::nvidia::isaac::VectorXi::Constant(1, 1)};
  }

  // Helper function to create a vector quantity
  static Quantity Vector(std::string entity, Measure measure, int dimension) {
    return Quantity{std::move(entity), measure, ::nvidia::isaac::VectorXi::Constant(1, dimension)};
  }

  // Equality operator
  bool operator==(const Quantity& other) const {
    return measure == other.measure && entity == other.entity && dimensions == other.dimensions;
  }

  // Total number of elements
  int getElementCount() const { return dimensions.prod(); }

  // The name of the entity for which the values was measured. Multiple quantities can be
  // measured per entity.
  std::string entity;
  // The measure, e.g. unit or type, of the value.
  Measure measure;
  // The dimensions of the value. Values can be multi-dimensional but most are scalars or vectors.
  ::nvidia::isaac::VectorXi dimensions;
};

// Exact comparison of quantities
struct QuantityEquality {
  bool operator()(const Quantity& lhs, const Quantity& rhs) const { return lhs == rhs; }
};

// Hash function for quantities using combination of hashes on elements
struct QuantityHash {
  std::size_t operator()(const Quantity& value) const {
    return std::hash<std::string>{}.operator()(value.entity) ^
           std::hash<int32_t>{}.operator()(static_cast<int32_t>(value.measure)) ^
           std::hash<int32_t>{}.operator()(static_cast<int32_t>(value.dimensions.size()));
  }
};

}  // namespace composite
}  // namespace isaac
}  // namespace nvidia
