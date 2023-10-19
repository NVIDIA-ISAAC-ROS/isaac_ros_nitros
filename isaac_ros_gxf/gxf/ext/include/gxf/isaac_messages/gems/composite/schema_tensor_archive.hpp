// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2020-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <array>
#include <cstdint>
#include <vector>

#include "common/expected.hpp"
#include "engine/core/tensor/tensor.hpp"
#include "gems/composite/schema.hpp"

namespace nvidia {
namespace isaac {

// An index map which can be used to map indices from one composite to another.
class CompositeIndexMap {
 public:
  // Error codes used by this data type
  enum struct Error {
    kQuantityNotFound,  // A quantity from the target schema was not found in the source schema.
  };

  // Computes an index map which can be used to associate elements in the target schema with
  // elements in the source schema. Will fail in case the mapping is not possible.
static Expected<CompositeIndexMap, Error> Create(const composite::Schema& source,
                                                 const composite::Schema& target);

  // Map a target index to the corresponding source index (see ComputeCompositeIndexMap)
  int32_t operator()(int32_t target) const { return indices_[target]; }

 private:
  // Private constructor to force initialization through static builder for RAII.
  CompositeIndexMap(int size) : indices_(size) {}

  // Stores the source index for each target index.
  // FIXME Use a stack allocated, bounded data type once available.
  std::vector<int32_t> indices_;
};

// Helper type to encode a composite into a tensor
template <typename K>
struct TensorArchiveEncoder {
  // Writes a value into the tensor at the given index.
  void operator()(int32_t index, K value) {
    composite_tensor(index) = value;
  }

  // The encoder writes data to this tensor.
  ::nvidia::isaac::CpuTensorView1<K> composite_tensor;
};

// Helper type to decode a composite from a tensor with the help of an index map
template <typename K>
struct TensorArchiveDecoder {
  // Reads a value from the tensor using an index from the *target* schema. The index map is used
  // to map target schema indices to indices used to access the tensor.
  void operator()(int32_t index, K& value) {
    value = composite_tensor(composite_index_map(index));
  }

  // The decoder reads data from this tensor.
  ::nvidia::isaac::CpuTensorConstView1<K> composite_tensor;
  // An index map which is used to map indices given to `operator()` to the correct location in
  // the tensor.
  CompositeIndexMap composite_index_map;
};

// Reads a composite state from a tensor
template <typename State>
void FromSchemaTensor(
    ::nvidia::isaac::CpuTensorConstView1<typename State::scalar_t> composite_tensor,
    const CompositeIndexMap& composite_index_map, State& state) {
  TensorArchiveDecoder<typename State::scalar_t> decoder{composite_tensor, composite_index_map};
  for (int32_t i = 0; i < state.size(); i++) {
    decoder(i, state[i]);
  }
}

// Writes a composite state to a tensor
template <typename State>
void ToSchemaTensor(const State& state,
                    ::nvidia::isaac::CpuTensorView1<typename State::scalar_t> composite_tensor) {
  TensorArchiveEncoder<typename State::scalar_t> encoder{composite_tensor};
  for (int32_t i = 0; i < state.size(); i++) {
    encoder(i, state[i]);
  }
}

}  // namespace isaac
}  // namespace nvidia
