// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2018-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <map>
#include <optional>
#include <string>
#include <utility>

#include "gems/core/optional.hpp"
#include "third_party/nlohmann/json.hpp"

namespace nvidia {
namespace isaac {

// We are using nlohmann::json as JSON
using Json = nlohmann::json;

namespace serialization {

// Loads and parses a JSON object from a file
Json LoadJsonFromFile(const std::string& filename);
// Parses a JSON object from a text string
// @deprecated
Json LoadJsonFromText(const std::string& text);
// Checks if the given file does not exist or is empty
bool IsFileEmpty(const std::string& filename);
// Appends two json objects into one: if the duplicate keys are found in the input objects,
// values are appended as array in the output object, second value is appended at back of array
// only works for the top level json. Combines the unqiue keys found in the input objects.
Json AppendJson(const Json& a, const Json& b);

// Loads and parses a JSON object from a file without ASSERT
// Returns nullopt in case of failure
std::optional<Json> TryLoadJsonFromFile(const std::string& filename);

// Writes JSON to a file
bool WriteJsonToFile(const std::string& filename, const Json& json);

// Merges two JSON objects into one: if duplicate keys are found in the input objects,
// the value of the second object is written to the merged object.
// Combines the unqiue keys found in the input objects.
Json MergeJson(const Json& a, const Json& b);

// Replace keys in the top level of json. In the key_map, the first value is the existing key to
// replace and second value is the new key with which to replace it.
// Returns the number of keys that were replaced (renaming a key to same name counts as a
// replacement).
int ReplaceJsonKeys(const std::map<std::string, std::string>& key_map, Json& json);

// Parses a JSON object from a text string
std::optional<Json> ParseJson(const std::string& text);
// Parses a JSON object from a string coming from an object such as a JSON or YAML.
// The string should be enclosed inside "" or '' and the internal strings have their delimiter
// escaped. For example: "{\"age\": 42}".
std::optional<Json> ParseJsonFromString(const std::string& str);

// Helper class for combining values in multiple JSON files and/or objects into one JSON object
class JsonOperator {
 public:
  using OperationFunction = std::function<Json(const Json&, const Json&)>;

  JsonOperator(OperationFunction operation_func) : operation_function_(operation_func) {}

  // Add JSON file to be combined
  JsonOperator& withFile(const std::string& json_filename) & {
    json_ = operation_function_(json_, LoadJsonFromFile(json_filename));
    return *this;
  }

  // Add JSON file to be combined
  JsonOperator&& withFile(const std::string& json_filename) && {
    json_ = operation_function_(json_, LoadJsonFromFile(json_filename));
    return std::move(*this);
  }

  // Add JSON object to be combined
  JsonOperator& withJson(const Json& json) & {
    json_ = operation_function_(json_, json);
    return *this;
  }

  // Add JSON object to be combined
  JsonOperator&& withJson(const Json& json) && {
    json_ = operation_function_(json_, json);
    return std::move(*this);
  }

  // Convert to JSON using a copy
  operator Json() const& { return json_; }

  // Convert to JSON using a move
  operator Json() && { return std::move(json_); }

 private:
  // Operation function to combine JSON
  OperationFunction operation_function_;
  // JSON being combined
  Json json_;
};

}  // namespace serialization
}  // namespace isaac
}  // namespace nvidia
