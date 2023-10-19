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

#include <string>
#include <utility>

#include "third_party/nlohmann/json.hpp"

#include "engine/core/image/image.hpp"

namespace nvidia {
namespace isaac {
namespace sight {

// Sight Operation Style
// Contains optional information to apply to the sight operation
struct SopStyle {
  // Creates empty style
  SopStyle() = default;

  // Creates a style with a color (valid javascript format: e.g. '#abc' '#123456' 'rgb(0, 1, 2)',
  // 'white', etc...)
  SopStyle(std::string color) {
    json_["c"] = std::move(color);
  }
  SopStyle(const char* color) {
    json_["c"] = color;
  }
  SopStyle(const Pixel3ub& col) {
    char buffer[8];
    std::snprintf(buffer, sizeof(buffer), "#%02x%02x%02x", col[0], col[1], col[2]);
    json_["c"] = std::string(buffer);
  }
  SopStyle(const Pixel4ub& col) : SopStyle(Pixel3ub{col[0], col[1], col[2]}) {
    json_["a"] = col[3]/255.0;
  }

  // Creates a style with a color and whether object should be filled or not.
  template <typename C>
  SopStyle(const C& color, bool fill) : SopStyle(color) {
    json_["f"] = fill;
  }

  // Creates a style with a color, whether object should be filled or not, and a default size.
  template <typename C>
  SopStyle(const C& color, bool fill, double size) : SopStyle(color, fill) {
    json_["s"] = size;
  }

  // Creates a style with a color, whether object should be filled or not, a default size, and an
  // alpha value to control transparency. If a Pixel4ub is provided for color, it's alpha component
  // will be overwritten by the alpha value provided as a double.
  template <typename C>
  SopStyle(const C& color, bool fill, double size, double alpha) : SopStyle(color, fill, size) {
    json_["a"] = alpha;
  }

 private:
  friend const nlohmann::json& ToJson(const SopStyle&);
  friend nlohmann::json ToJson(SopStyle&&);

  nlohmann::json json_;
};

// Returns the json of a SopStyle
const nlohmann::json& ToJson(const SopStyle&);
nlohmann::json ToJson(SopStyle&&);

}  // namespace sight
}  // namespace isaac
}  // namespace nvidia
