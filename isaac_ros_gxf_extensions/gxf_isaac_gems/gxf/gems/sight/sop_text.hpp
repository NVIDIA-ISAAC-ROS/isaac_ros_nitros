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

#include <optional>
#include <string>
#include <utility>

#include "gems/core/math/types.hpp"
#include "gems/sight/sop_serializer.hpp"

namespace nvidia {
namespace isaac {
namespace sight {

// Sight Operation Text
// Can be used to display text at a given position
class SopText : public SopSerializer {
 public:
  // Creates empty SopText
  SopText() = default;
  SopText(SopText&&) = default;
  ~SopText() override = default;

  // Creates a text with not other information (will be rendered at (0, 0))
  SopText(std::string text);

  // Creates a style with a color and wether object should be filled or not.
  template <typename K>
  SopText(std::string text, const ::nvidia::isaac::Vector2<K>& pos) : SopText(std::move(text)) {
    position_ = pos.template cast<float>();
  }

  // Creates a SopText setup to center the text.
  template <typename K>
  SopText(std::string text, const ::nvidia::isaac::Vector2<K>& pos, bool center)
      : SopText(std::move(text), pos) {
    center_ = center;
  }

  // Construct a sop by calling fromBinary
  SopText(BufferSerialization& buffer);

  bool toBinary(BufferSerialization& buffer) const override;
  bool fromBinary(BufferSerialization& buffer) override;

 private:
  std::string text_ = "";
  bool center_ = false;
  std::optional<::nvidia::isaac::Vector2f> position_;
};

}  // namespace sight
}  // namespace isaac
}  // namespace nvidia
