// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#ifndef NVIDIA_GXF_STD_CPU_THREAD_HPP
#define NVIDIA_GXF_STD_CPU_THREAD_HPP

#include <string>
#include <vector>

#include "gxf/core/component.hpp"

namespace nvidia {
namespace gxf {

// Real-time scheduling policies supported by POSIX and Linux kernel
enum class SchedulingPolicy : int32_t {
  kFirstInFirstOut = 1,  // SCHED_FIFO supported by POSIX and Linux kernel
  kRoundRobin = 2,  // SCHED_RR supported by POSIX and Linux kernel
  kDeadline = 6  // SCHED_DEADLINE supported by Linux kernel
};

// Custom parameter parser for SchedulingPolicy
template <>
struct ParameterParser<SchedulingPolicy> {
  static Expected<SchedulingPolicy> Parse(gxf_context_t context, gxf_uid_t component_uid,
                                         const char* key, const YAML::Node& node,
                                         const std::string& prefix) {
    const std::string value = node.as<std::string>();
    if (strcmp(value.c_str(), "SCHED_FIFO") == 0) {
      return SchedulingPolicy::kFirstInFirstOut;
    }
    if (strcmp(value.c_str(), "SCHED_RR") == 0) {
      return SchedulingPolicy::kRoundRobin;
    }
    if (strcmp(value.c_str(), "SCHED_DEADLINE") == 0) {
      return SchedulingPolicy::kDeadline;
    }
    GXF_LOG_ERROR("Invalid scheduling policy: %s", value.c_str());
    return Unexpected{GXF_ARGUMENT_OUT_OF_RANGE};
  }
};

// Custom parameter wrapper for SchedulingPolicy
template<>
struct ParameterWrapper<SchedulingPolicy> {
  static Expected<YAML::Node> Wrap(gxf_context_t context, const SchedulingPolicy& value) {
    YAML::Node node(YAML::NodeType::Scalar);
    switch (value) {
      case SchedulingPolicy::kFirstInFirstOut: {
        node = std::string("SCHED_FIFO");
        break;
      }
      case SchedulingPolicy::kRoundRobin: {
        node = std::string("SCHED_RR");
        break;
      }
      case SchedulingPolicy::kDeadline: {
        node = std::string("SCHED_DEADLINE");
        break;
      }
      default:
        GXF_LOG_ERROR("Invalid scheduling policy: %d", static_cast<int32_t>(value));
        return Unexpected{GXF_PARAMETER_OUT_OF_RANGE};
    }
    return node;
  }
};

class CPUThread : public Component {
 public:
  gxf_result_t registerInterface(Registrar* registrar) override;

  bool pinned() const {
    return pin_entity_;
  }

  std::vector<uint32_t> pinCores() const {
    const auto maybe_pin_cores = pin_cores_.try_get();
    if (maybe_pin_cores) {
      return maybe_pin_cores.value();
    }
    return {};
  }

  Expected<SchedulingPolicy> schedPolicy() const {
    return sched_policy_.try_get();
  }

  Expected<uint32_t> schedPriority() const {
    return sched_priority_.try_get();
  }

  Expected<uint64_t> schedRuntime() const {
    return sched_runtime_.try_get();
  }

  Expected<uint64_t> schedDeadline() const {
    return sched_deadline_.try_get();
  }

  Expected<uint64_t> schedPeriod() const {
    return sched_period_.try_get();
  }

  // Helper method to check if real-time scheduling is used
  bool isRealtime() const {
    const auto maybe_sched_policy = sched_policy_.try_get();
    if (maybe_sched_policy) {
      auto sched_policy_value = maybe_sched_policy.value();
      return sched_policy_value == SchedulingPolicy::kFirstInFirstOut ||
             sched_policy_value == SchedulingPolicy::kRoundRobin ||
             sched_policy_value == SchedulingPolicy::kDeadline;
    }
    return false;
  }

 private:
  // Keep track of whether or not the component should be pinned to a worker thread
  Parameter<bool> pin_entity_;
  // CPU core IDs to pin the worker thread to (empty means no core pinning)
  Parameter<std::vector<uint32_t>> pin_cores_;

  // Real-time scheduling parameters (optional)
  // Real-time scheduling policy (kFirstInFirstOut, kRoundRobin, kDeadline)
  Parameter<SchedulingPolicy> sched_policy_;
  // Thread priority (only for kFirstInFirstOut and kRoundRobin)
  Parameter<uint32_t> sched_priority_;
  // Expected worst case execution time in nanoseconds (only for kDeadline)
  Parameter<uint64_t> sched_runtime_;
  // Relative deadline in nanoseconds (only for kDeadline)
  Parameter<uint64_t> sched_deadline_;
  // Period in nanoseconds (only for kDeadline)
  Parameter<uint64_t> sched_period_;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_STD_CPU_THREAD_HPP
