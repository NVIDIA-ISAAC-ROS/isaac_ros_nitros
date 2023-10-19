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

#include "engine/core/math/types.hpp"
#include "engine/gems/serialization/json_formatter.hpp"

namespace nvidia {
namespace isaac {
namespace sight {

// Sight Operation Text
// Can be used to display text at a given position
// TODO: Add more styles and options
struct SopText {
  // Creates a text with not other information (will be rendered at (0, 0))
  SopText(std::string text) {
    json_["t"] = "text";
    json_["text"] = std::move(text);
  }
  SopText(const char* text) {
    json_["t"] = "text";
    json_["text"] = text;
  }

  // Creates a style with a color and wether object should be filled or not.
  template <typename Text, typename K, int N>
  SopText(const Text& text, const Vector<K, N>& pos) : SopText(text) {
    serialization::Set(json_["p"], pos);
  }

  // Creates a SopText setup to center the text.
  template <typename... Args>
  static SopText Centered(Args... args) {
    SopText sop(args...);
    sop.json_["center"] = true;
    return sop;
  }

 private:
  friend const nlohmann::json& ToJson(const SopText&);
  friend nlohmann::json ToJson(SopText&&);

  // Creates empty style
  SopText() = default;

  nlohmann::json json_;
};

// Returns the json of a SopText
const nlohmann::json& ToJson(const SopText&);
nlohmann::json ToJson(SopText&&);

}  // namespace sight
}  // namespace isaac
}  // namespace nvidia
