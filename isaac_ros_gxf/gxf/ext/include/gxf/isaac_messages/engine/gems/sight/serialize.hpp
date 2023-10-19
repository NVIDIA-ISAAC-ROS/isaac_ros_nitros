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

#include <array>
#include <vector>

#include "engine/gems/geometry/types.hpp"
#include "engine/gems/serialization/json.hpp"
#include "engine/gems/serialization/json_formatter.hpp"

namespace nvidia {
namespace isaac {
namespace sight {

// Returns a json describing a Cuboid
template<typename K, int N>
Json ToJson(const geometry::NCuboid<K, N>& cuboid) {
  Json json;
  json["t"] = "cube";
  serialization::Set(json["a"], cuboid.min());
  serialization::Set(json["b"], cuboid.max());
  return json;
}

// Returns a json describing a Sphere
template<typename K, int N>
Json ToJson(const geometry::NSphere<K, N>& sphere) {
  Json json;
  json["t"] = "sphr";
  serialization::Set(json["c"], sphere.center);
  json["r"] = sphere.radius;
  return json;
}

// Returns a json describing a Line
template<typename K, int N>
Json ToJson(const geometry::LineSegment<K, N>& seg) {
  Json json;
  json["t"] = "line";
  {
    Json tmp;
    serialization::Set(tmp, seg.a());
    json["p"].push_back(tmp);
  }
  {
    Json tmp;
    serialization::Set(tmp, seg.b());
    json["p"].push_back(tmp);
  }
  return json;
}

// Returns a json describing a polyline
template<typename K, int N>
Json ToJson(const std::vector<Vector<K, N>>& poly) {
  Json json;
  json["t"] = "pnts";
  serialization::Set(json["p"], poly);
  return json;
}
template<typename K, int N, size_t M>
Json ToJson(const std::array<Vector<K, N>, M>& poly) {
  Json json;
  json["t"] = "pnts";
  serialization::Set(json["p"], poly);
  return json;
}

// Returns a json describing a polygon
template<typename K>
Json ToJson(const geometry::Polygon2<K>& polygon) {
  Json json;
  json["t"] = "line";
  serialization::Set(json["p"], polygon.points);
  {
    Json tmp;
    serialization::Set(tmp, polygon.points.front());
    json["p"].push_back(tmp);
  }
  return json;
}
template<typename K>
Json ToJson(const geometry::Polygon3<K>& polygon) {
  Json json;
  json["t"] = "line";
  serialization::Set(json["p"], polygon.points);
  {
    Json tmp;
    serialization::Set(tmp, polygon.points.front());
    json["p"].push_back(tmp);
  }
  return json;
}

// Returns a json describing a point
template<typename K, int N>
Json ToJson(const Vector<K, N>& p) {
  Json json;
  json["t"] = "pnts";
  Json tmp;
  serialization::Set(tmp, p);
  json["p"].push_back(tmp);
  return json;
}

}  // namespace sight
}  // namespace isaac
}  // namespace nvidia
