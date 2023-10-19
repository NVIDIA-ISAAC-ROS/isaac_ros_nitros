// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2018-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <type_traits>

#include "engine/core/assert.hpp"
#include "engine/core/buffers/traits.hpp"
#include "engine/core/byte.hpp"

namespace nvidia {
namespace isaac {

// Copies raw memory from one buffer to another buffer
void CopyArrayRaw(const void* source, BufferStorageMode source_storage,
                  void* target, BufferStorageMode target_storage, size_t size);

// Copies data between two buffers layout as matrices
// For both source and target buffer the pointer and the row stride between rows is given.
// `row_size` indicates the number of bytes which are actually filled with data. Bytes which are
// not filled with used data might not be copied.
void CopyMatrixRaw(const void* source, size_t source_stride, BufferStorageMode source_storage,
                   void* target, size_t target_stride, BufferStorageMode target_storage,
                   size_t rows, size_t row_size);

// Copies memory from one buffer to another buffer
template <typename Source, typename Target>
void CopyArray(const Source& source, Target& target) {
  // Asserts that the buffers have the same size
  ASSERT(source.size() == target.size(), "size mismatch: %zu vs %zu", source.size(), target.size());
  // Copy the bytes
  auto source_view = View(source).template reinterpret<const byte>();
  auto target_view = View(target).template reinterpret<byte>();
  CopyArrayRaw(reinterpret_cast<const void*>(source_view.begin()),
               BufferTraits<Source>::kStorageMode,
               reinterpret_cast<void*>(target_view.begin()),
               BufferTraits<Target>::kStorageMode,
               source_view.size());
}

}  // namespace isaac
}  // namespace nvidia
