// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2021-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "gems/composite/composite_view.hpp"
#include "gems/composite/schema.hpp"

namespace nvidia {
namespace isaac {

// Describes each beam in a flatscan
struct FlatscanIndices {
  enum {
    // Angle of the beam
    kAngle,
    // Start of the free range
    kRangeStart,
    // End of the free range
    kRangeEnd,
    // Visibility range of the flat ray, e.g., the furthest distance the ray could reach within
    // the height slice without any obstacles
    kVisibilityRange,
    // The relative time for this beam to the message timestamp, unit in seconds
    // beam_absolute_time = message_time + relative_time
    kRelativeTime,
    kSize
  };
};

inline composite::Schema FlatscanCompositeSchema() {
  return composite::Schema({
      composite::Quantity::Scalar("angle", composite::Measure::kRotation),
      composite::Quantity::Scalar("range_start", composite::Measure::kPosition),
      composite::Quantity::Scalar("range_end", composite::Measure::kPosition),
      composite::Quantity::Scalar("visibility_range", composite::Measure::kPosition),
      composite::Quantity::Scalar("relative_time", composite::Measure::kTime),
  });
}

template <typename K, typename Base>
struct FlatscanImmutableInterface : public Base {
  using Base::Base;
  using Base::operator=;

  K angle() const { return get<FlatscanIndices::kAngle>(*this); }
  K range_start() const { return get<FlatscanIndices::kRangeStart>(*this); }
  K range_end() const { return get<FlatscanIndices::kRangeEnd>(*this); }
  K visibility_range() const { return get<FlatscanIndices::kVisibilityRange>(*this); }
  K relative_time() const { return get<FlatscanIndices::kRelativeTime>(*this); }
};

template <typename K, typename Base>
struct FlatscanMutableInterface : public FlatscanImmutableInterface<K, Base> {
  using FlatscanImmutableInterface<K, Base>::FlatscanImmutableInterface;
  using FlatscanImmutableInterface<K, Base>::operator=;

  K& angle() { return get<FlatscanIndices::kAngle>(*this); }
  K& range_start() { return get<FlatscanIndices::kRangeStart>(*this); }
  K& range_end() { return get<FlatscanIndices::kRangeEnd>(*this); }
  K& visibility_range() { return get<FlatscanIndices::kVisibilityRange>(*this); }
  K& relative_time() { return get<FlatscanIndices::kRelativeTime>(*this); }
};

template <typename K>
using FlatscanConstView = CompositeConstView<K, FlatscanIndices, FlatscanImmutableInterface>;

template <typename K>
using FlatscanView = CompositeView<K, FlatscanIndices, FlatscanMutableInterface>;

template <typename K>
using Flatscan = Composite<K, FlatscanIndices, FlatscanMutableInterface>;

}  // namespace isaac
}  // namespace nvidia
