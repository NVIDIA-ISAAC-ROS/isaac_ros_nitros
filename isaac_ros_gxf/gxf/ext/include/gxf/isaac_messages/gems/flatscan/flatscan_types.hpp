/*
Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "gems/composite/composite_view.hpp"
#include "packages/composite/gems/schema.hpp"

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

inline ::isaac::composite::Schema FlatscanCompositeSchema() {
  return ::isaac::composite::Schema({
      ::isaac::composite::Quantity::Scalar("angle", ::isaac::composite::Measure::kRotation),
      ::isaac::composite::Quantity::Scalar("range_start", ::isaac::composite::Measure::kPosition),
      ::isaac::composite::Quantity::Scalar("range_end", ::isaac::composite::Measure::kPosition),
      ::isaac::composite::Quantity::Scalar(
          "visibility_range", ::isaac::composite::Measure::kPosition),
      ::isaac::composite::Quantity::Scalar(
          "relative_time", ::isaac::composite::Measure::kTime),
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
