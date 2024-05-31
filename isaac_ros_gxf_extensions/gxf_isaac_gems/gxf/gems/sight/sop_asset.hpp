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
#pragma once

#include <string>

#include "gems/sight/sop_serializer.hpp"

namespace nvidia {
namespace isaac {
namespace sight {

// Sight Operation Asset.
// TODO(bbutin): Right it now it contains only a name to an asset, later we need to add information
// about the asset as well.
class SopAsset : public SopSerializer {
 public:
  // Creates empty SopAsset
  SopAsset() = default;
  SopAsset(SopAsset&&) = default;
  ~SopAsset() override = default;

  // Creates an asset with a name
  SopAsset(std::string text);

  // Construct a sop by calling fromBinary
  SopAsset(BufferSerialization& buffer);

  bool toBinary(BufferSerialization& buffer) const override;
  bool fromBinary(BufferSerialization& buffer) override;

 private:
  std::string name_ = "";
};

}  // namespace sight
}  // namespace isaac
}  // namespace nvidia
