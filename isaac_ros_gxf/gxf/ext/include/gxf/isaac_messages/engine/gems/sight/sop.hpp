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

#include <utility>

#include "engine/core/image/image.hpp"
#include "engine/core/math/types.hpp"
#include "engine/core/optional.hpp"
#include "engine/gems/serialization/json.hpp"
#include "engine/gems/sight/named_sop.hpp"
#include "engine/gems/sight/serialize.hpp"
#include "engine/gems/sight/sop_asset.hpp"
#include "engine/gems/sight/sop_image.hpp"
#include "engine/gems/sight/sop_mesh.hpp"
#include "engine/gems/sight/sop_point_cloud.hpp"
#include "engine/gems/sight/sop_style.hpp"
#include "engine/gems/sight/sop_text.hpp"
#include "engine/gems/sight/sop_transform.hpp"

namespace nvidia {
namespace isaac {
namespace sight {

class Sop;

// Determine if a type is "callable", for example if it is a function object
template <typename...> using void_t = void;
template <typename, typename = void> struct is_sop_callback : std::false_type {};
template <typename T>
struct is_sop_callback<T, void_t<std::result_of_t<T(Sop&)>>>
    : std::is_same<void, std::result_of_t<T(Sop&)>> {};
template <typename T> constexpr bool is_sop_callback_v = is_sop_callback<T>::value;

// Contains sight operation as a Tree structure:
//  - Each internal node contains a potential style (default for its children), a transformation to
//    apply before rendering the children
//  - Each leaf contains a simple primitive (or SopImage) to be rendered using the current default
//    style and using the composition of all the transformation above.
class Sop {
 public:
  // Simple constructor
  Sop() = default;
  Sop(const Sop&) = delete;
  Sop(Sop&&) = default;

  // Constructor from Json input.
  // Converts from "named SOP" format to "standard SOP" format as described in the documentation for
  // "convertFromNamedSop()".
  Sop(const Json& json) {
    ConvertFromNamedSop(json, json_);
  }

  // Constructor from JsonMerger input.
  // Converts from "named SOP" format to "standard SOP" format as described in the documentation for
  // "convertFromNamedSop()".
  Sop(const serialization::JsonMerger& json_merger) {
    Json json = json_merger;
    ConvertFromNamedSop(json, json_);
  }

  // Constructor from JsonMerger rvalue reference as input.
  // Converts from "named SOP" format to "standard SOP" format as described in the documentation for
  // "convertFromNamedSop()".
  Sop(serialization::JsonMerger&& json_merger) {
    Json json = std::move(json_merger);
    ConvertFromNamedSop(json, json_);
  }

  // Assignment from Json.
  // Converts from "named SOP" format to "standard SOP" format as described in the documentation for
  // "convertFromNamedSop()".
  Sop& operator=(const Json& json) {
    ConvertFromNamedSop(json, json_);
    return *this;
  }

  // Assignment from JsonMerger.
  // Converts from "named SOP" format to "standard SOP" format as described in the documentation for
  // "convertFromNamedSop()"
  Sop& operator=(const serialization::JsonMerger& json_merger) {
    Json json = json_merger;
    ConvertFromNamedSop(json, json_);
    return *this;
  }

  // Assignment from JsonMerger rvalue reference.
  // Converts from "named SOP" format to "standard SOP" format as described in the documentation for
  // "convertFromNamedSop()".
  Sop& operator=(serialization::JsonMerger&& json_merger) {
    Json json = std::move(json_merger);
    ConvertFromNamedSop(json, json_);
    return *this;
  }

  // Add a sop
  void add(Sop sop) {
    json_["d"].push_back(sop.moveJson());
  }

  // Add a new primitive to the list of children
  template <typename Primitive, std::enable_if_t<!is_sop_callback_v<Primitive>, int> = 0>
  void add(const Primitive& primitive) {
    json_["d"].push_back(::nvidia::isaac::sight::ToJson(primitive));
  }

  // Specific implementation for Image.
  template <typename K, int N, typename Container>
  void add(const ImageBase<K, N, Container>& img) {
    add(SopImage::Jpg(img));
  }
  template <typename K, int N, typename Container>
  void add(const ImageBase<K, N, Container>& img, bool use_png) {
    add(use_png ? SopImage::Png(img) : SopImage::Jpg(img));
  }

  // Add a primitive with a given transformation
  template<class Primitive>
  void add(const Primitive& primitive, const SopTransform& transform) {
    Sop sop;
    sop.transform = transform;
    sop.add(primitive);
    json_["d"].push_back(sop.moveJson());
  }

  // Add a primitive with a given style
  template<class Primitive>
  void add(const Primitive& primitive, const SopStyle& style) {
    Sop sop;
    sop.style = style;
    sop.add(primitive);
    json_["d"].push_back(sop.moveJson());
  }

  // Add a primitive with a given transformation and style
  template<class Primitive>
  void add(const Primitive& primitive, const SopStyle& style, const SopTransform& transform) {
    Sop sop;
    sop.transform = transform;
    sop.style = style;
    sop.add(primitive);
    json_["d"].push_back(sop.moveJson());
  }

  // Add a new Sop using a callback function:
  //  sop.add([&](sight::Sop& sop) {
  //     ...
  //  });
  template <typename F, std::enable_if_t<is_sop_callback_v<F>, int> = 0>
  void add(F f) {
    Sop sop;
    f(sop);
    json_["d"].emplace_back(sop.moveJson());
  }

  // Make a sop static, it will be rendered at the latest time from the PoseTree.
  void makeStatic() {
    json_["f"] = true;
  }

  // Move the object to prevent copy. json_ will be empty afterwards meaning the content of this
  // Sop will effectively be empty.
  Json&& moveJson() {
    updateJSON();
    return std::move(json_);
  }

  // Returns the current json
  const Json& toJson() {
    updateJSON();
    return json_;
  }

  // Optional style to be apply to the children
  std::optional<SopStyle> style;
  // Optional transformation to apply to the the subtree
  std::optional<SopTransform> transform;

 private:
  // Updates the json with the information about the style and transformation
  void updateJSON() {
    if (style) json_["s"] = ToJson(*style);
    if (transform) json_["p"] = ToJson(*transform);
    json_["t"] = "sop";
  }

  Json json_;
};

}  // namespace sight
}  // namespace isaac
}  // namespace nvidia
