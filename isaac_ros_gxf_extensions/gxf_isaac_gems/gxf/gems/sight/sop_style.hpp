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

#include "gems/core/image/image.hpp"
#include "gems/sight/sop_serializer.hpp"

namespace nvidia {
namespace isaac {
namespace sight {

// Sight Operation Style
// Contains optional information to apply to the sight operation
struct SopStyle : public SopSerializer {
  // Creates empty style
  SopStyle() = default;
  ~SopStyle() override = default;

  // Creates a style with a color (valid javascript format: e.g. '#abc' '#123456' 'rgb(0, 1, 2)',
  // 'white', etc...)
  SopStyle(std::string color);
  SopStyle(const char* color);
  SopStyle(const ::nvidia::isaac::Pixel3ub& col);
  SopStyle(const ::nvidia::isaac::Pixel4ub& col);

  // Helper == operator for testing deserialize.
  bool operator==(const SopStyle& style) const;

  // Creates a style with a color and whether object should be filled or not.
  template <typename C>
  SopStyle(const C& color, bool fill) : SopStyle(color) {
    fill_ = fill;
  }

  // Creates a style with a color, whether object should be filled or not, and a default size.
  template <typename C>
  SopStyle(const C& color, bool fill, double size) : SopStyle(color, fill) {
    size_ = size;
  }

  // Creates a style with a color, whether object should be filled or not, a default size, and an
  // alpha value to control transparency. If a Pixel4ub is provided for color, it's alpha component
  // will be overwritten by the alpha value provided as a double.
  template <typename C>
  SopStyle(const C& color, bool fill, double size, double alpha) : SopStyle(color, fill, size) {
    alpha_ = static_cast<uint8_t>(std::clamp(256.0 * alpha, 0.0, 255.0));
  }

  bool toBinary(BufferSerialization& buffer) const override;

  bool fromBinary(BufferSerialization& buffer) override;

 private:
  std::optional<std::string> color_str_;
  std::optional<::nvidia::isaac::Pixel3ub> color_;
  std::optional<uint8_t> alpha_;
  std::optional<bool> fill_;
  std::optional<float> size_;
};

}  // namespace sight
}  // namespace isaac
}  // namespace nvidia
