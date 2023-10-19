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

namespace nvidia {
namespace isaac {
namespace sight {

// Sight Operation Asset.
// TODO: Right it now it contains only a name to an asset, later we need to add information about
// the asset as well.
class SopAsset {
 public:
  // Delete copy constructor
  SopAsset(const SopAsset&) = delete;
  SopAsset(SopAsset&&) = default;

  static SopAsset FromName(std::string name) {
    SopAsset asset;
    asset.json_["t"] = "asset";
    asset.json_["n"] = std::move(name);
    return asset;
  }

  // Returns name of the SopAsset.
  std::string name() const {
    return json_["n"].get<std::string>();
  }

 private:
  friend const nlohmann::json& ToJson(const SopAsset&);
  friend nlohmann::json ToJson(SopAsset&&);

  // Private to allow construction from the static function
  SopAsset() = default;

  nlohmann::json json_;
};

// Returns the json of a SopAsset
const nlohmann::json& ToJson(const SopAsset&);
nlohmann::json ToJson(SopAsset&&);

}  // namespace sight
}  // namespace isaac
}  // namespace nvidia
