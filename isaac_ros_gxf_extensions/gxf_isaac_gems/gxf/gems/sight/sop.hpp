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

#include <memory>
#include <optional>
#include <type_traits>
#include <utility>
#include <vector>

#include "gems/sight/sop_geometry.hpp"
#include "gems/sight/sop_serializer.hpp"
#include "gems/sight/sop_style.hpp"
#include "gems/sight/sop_transform.hpp"

namespace nvidia {
namespace isaac {
namespace sight {

// Contains sight operation as a Tree structure:
//  - Each internal node contains a potential style (default for its children), a transformation to
//    apply before rendering the children
//  - Each leaf contains a simple primitive (or SopImage) to be rendered using the current default
//    style and using the composition of all the transformation above.
class Sop : public SopSerializer {
 public:
  // Simple constructor
  Sop() = default;
  Sop(Sop&&) = default;
  Sop(const Sop&) = delete;

  // Add a SopSerializer
  Sop(SopSerializer* sop) {
    list_.emplace_back(sop);
  }

  // Add a primitive without a transform or style
  template <class Primitive, std::enable_if_t<std::is_base_of_v<SopSerializer, Primitive>, int> = 0>
  void add(Primitive primitive) {
    list_.emplace_back(new Primitive(std::move(primitive)));
  }

  // Add a primitive with a given transform
  template <class Primitive, std::enable_if_t<std::is_base_of_v<SopSerializer, Primitive>, int> = 0>
  void add(Primitive primitive, const SopTransform& transform) {
    Sop* sop = new Sop(new Primitive(std::move(primitive)));
    sop->transform = transform;
    list_.emplace_back(static_cast<SopSerializer*>(sop));
  }

  // Add a primitive with a given style
  template <class Primitive, std::enable_if_t<std::is_base_of_v<SopSerializer, Primitive>, int> = 0>
  void add(Primitive primitive, const SopStyle& style) {
    Sop* sop = new Sop(new Primitive(std::move(primitive)));
    sop->style = style;
    list_.emplace_back(static_cast<SopSerializer*>(sop));
  }

  // Add a primitive with a given transform and style
  template <class Primitive, std::enable_if_t<std::is_base_of_v<SopSerializer, Primitive>, int> = 0>
  void add(Primitive primitive, const SopStyle& style, const SopTransform& transform) {
    Sop* sop = new Sop(new Primitive(std::move(primitive)));
    sop->style = style;
    sop->transform = transform;
    list_.emplace_back(static_cast<SopSerializer*>(sop));
  }

  // Add a primitive with a given transform
  template <class Primitive, typename... Args>
  void add(const Args&... args) {
    list_.emplace_back(new Primitive(args...));
  }

  // Add a primitive with a given transform
  template <class Primitive, typename... Args>
  void add(const SopTransform& transform, const Args&... args) {
    Sop* sop = new Sop(new Primitive(args...));
    sop->transform = transform;
    list_.emplace_back(static_cast<SopSerializer*>(sop));
  }

  // Add a primitive with a given style
  template <class Primitive, typename... Args>
  void add(const SopStyle& style, const Args&... args) {
    Sop* sop = new Sop(new Primitive(args...));
    sop->style = style;
    list_.emplace_back(static_cast<SopSerializer*>(sop));
  }

  // Add a primitive with a given transform and style
  template <class Primitive, typename... Args>
  void add(const SopStyle& style, const SopTransform& transform, const Args&... args) {
    Sop* sop = new Sop(new Primitive(args...));
    sop->style = style;
    sop->transform = transform;
    list_.emplace_back(static_cast<SopSerializer*>(sop));
  }

  template <typename F, std::enable_if_t<std::is_invocable_v<F, Sop&>, int> = 0>
  void add(F f) {
    Sop* sop = new Sop();
    f(*sop);
    list_.emplace_back(static_cast<SopSerializer*>(sop));
  }

  // Make a sop static, it will be rendered at the latest time from the PoseTree.
  void makeStatic();

  uint8_t key() const;

  bool toBinary(BufferSerialization& buffer) const override;

  bool fromBinary(BufferSerialization& buffer) override;

  // Optional style to be apply to the children
  std::optional<SopStyle> style;
  // Optional transformation to apply to the the subtree
  std::optional<SopTransform> transform;

 private:
  bool static_ = false;
  std::vector<std::unique_ptr<SopSerializer>> list_;
};

}  // namespace sight
}  // namespace isaac
}  // namespace nvidia
