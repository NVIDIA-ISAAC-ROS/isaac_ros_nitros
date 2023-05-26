/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <map>
#include <string>
#include <utility>

#include "engine/core/optional.hpp"
#include "third_party/nlohmann/json.hpp"

namespace isaac {

// We are using nlohmann::json as JSON
using Json = nlohmann::json;

namespace serialization {

// Loads and parses a JSON object from a file
Json LoadJsonFromFile(const std::string& filename);
// Parses a JSON object from a text string
// @deprecated
Json LoadJsonFromText(const std::string& text);

// Loads and parses a JSON object from a file without ASSERT
// Returns nullopt in case of failure
std::optional<Json> TryLoadJsonFromFile(const std::string& filename);

// Writes JSON to a file
bool WriteJsonToFile(const std::string& filename, const Json& json);

// Merges two JSON objects into one
Json MergeJson(const Json& a, const Json& b);

// Replace keys in the top level of json. In the key_map, the first value is the existing key to
// replace and second value is the new key with which to replace it.
// Returns the number of keys that were replaced (renaming a key to same name counts as a
// replacement).
int ReplaceJsonKeys(const std::map<std::string, std::string>& key_map, Json& json);

// Parses a JSON object from a text string
std::optional<Json> ParseJson(const std::string& text);

// Helper class for merging multiple JSON files and/or objects into one JSON object
class JsonMerger {
 public:
  JsonMerger() {}

  // Add JSON file to be merged
  JsonMerger& withFile(const std::string& json_filename) & {
    json_ = MergeJson(json_, LoadJsonFromFile(json_filename));
    return *this;
  }

  // Add JSON file to be merged
  JsonMerger&& withFile(const std::string& json_filename) && {
    json_ = MergeJson(json_, LoadJsonFromFile(json_filename));
    return std::move(*this);
  }

  // Add JSON object to be merged
  JsonMerger& withJson(const Json& json) & {
    json_ = MergeJson(json_, json);
    return *this;
  }

  // Add JSON object to be merged
  JsonMerger&& withJson(const Json& json) && {
    json_ = MergeJson(json_, json);
    return std::move(*this);
  }

  // Convert to JSON using a copy
  operator Json() const& { return json_; }

  // Convert to JSON using a move
  operator Json() && { return std::move(json_); }

 private:
  // JSON being merged
  Json json_;
};

}  // namespace serialization
}  // namespace isaac
