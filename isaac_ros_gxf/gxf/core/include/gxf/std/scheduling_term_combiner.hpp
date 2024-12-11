// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#ifndef NVIDIA_GXF_STD_SCHEDULING_TERM_COMBINER_HPP
#define NVIDIA_GXF_STD_SCHEDULING_TERM_COMBINER_HPP

#include <stdint.h>

#include <algorithm>

#include "gxf/std/scheduling_term.hpp"

namespace nvidia {
namespace gxf {

/**
 * @brief Base class for scheduling term combiner
 *
 * scheduling term combiners can be used to create complex execution patterns by interpreting
 * combinations of scheduling terms differently
 */
class SchedulingTermCombiner : public Component {
 public:
  // Function to combine two SchedulingCondition types
  virtual SchedulingCondition combine(SchedulingCondition a, SchedulingCondition b) = 0;

  // Get the list of scheduling terms in an entity being governed by the combiner
  virtual FixedVector<Handle<SchedulingTerm>, kMaxComponents> getTermList() const = 0;
};

/**
 * @brief OR combiners simulate the bitwise OR operation when combining scheduling conditions
 *
 */
class OrSchedulingTermCombiner : public SchedulingTermCombiner {
 public:
  gxf_result_t registerInterface(Registrar* registrar) override {
    Expected<void> result;
    result &= registrar->parameter(terms_, "terms", "SchedulingTerms",
                                 "The list of scheduling terms to be combined using OR operation");
    return GXF_SUCCESS;
  }

  SchedulingCondition combine(SchedulingCondition a, SchedulingCondition b) override {
    // "never" has the highest significance
    if (a.type == SchedulingConditionType::NEVER || b.type == SchedulingConditionType::NEVER) {
      return {SchedulingConditionType::NEVER, 0};
    }

    // If both are ready, choose the max target timestamp
    if (a.type == SchedulingConditionType::READY && b.type == SchedulingConditionType::READY) {
      return {SchedulingConditionType::READY, std::max(a.target_timestamp, b.target_timestamp)};
    }

    // Check if either one of them is ready
    if (a.type == SchedulingConditionType::READY) {
      return {SchedulingConditionType::READY, a.target_timestamp};
    } else if (b.type == SchedulingConditionType::READY) {
      return {SchedulingConditionType::READY, b.target_timestamp};
    }

    // "wait event" has the third highest significance
    if (a.type == SchedulingConditionType::WAIT_EVENT ||
        b.type == SchedulingConditionType::WAIT_EVENT) {
      return {SchedulingConditionType::WAIT_EVENT, 0};
    }

    // "wait time" events are combined so that the maximum time is returned
    if (a.type == SchedulingConditionType::WAIT_TIME &&
        b.type == SchedulingConditionType::WAIT_TIME) {
      return {SchedulingConditionType::WAIT_TIME, std::max(a.target_timestamp, b.target_timestamp)};
    }

    // Check if atleast one of them have a target time
    if (a.type == SchedulingConditionType::WAIT_TIME) {
      return a;
    } else if (b.type == SchedulingConditionType::WAIT_TIME) {
      return b;
    }

    // The only remaining case is that both are wait
    return {SchedulingConditionType::WAIT, 0};
  }

FixedVector<Handle<SchedulingTerm>, kMaxComponents> getTermList() const {
  return terms_;
}

Parameter<FixedVector<Handle<SchedulingTerm>, kMaxComponents>> terms_;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_STD_SCHEDULING_TERM_COMBINER_HPP
