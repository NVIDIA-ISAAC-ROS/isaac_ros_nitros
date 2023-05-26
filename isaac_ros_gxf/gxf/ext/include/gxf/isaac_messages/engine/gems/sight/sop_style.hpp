/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>
#include <utility>

#include "third_party/nlohmann/json.hpp"

#include "engine/core/image/image.hpp"

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
