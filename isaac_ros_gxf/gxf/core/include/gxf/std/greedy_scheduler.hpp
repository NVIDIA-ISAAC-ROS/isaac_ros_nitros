// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2020-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef NVIDIA_GXF_STD_GREEDY_SCHEDULER_HPP_
#define NVIDIA_GXF_STD_GREEDY_SCHEDULER_HPP_

#include <algorithm>
#include <atomic>
#include <memory>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "common/fixed_vector.hpp"
#include "common/logger.hpp"
#include "gxf/core/component.hpp"
#include "gxf/core/entity.hpp"
#include "gxf/core/handle.hpp"
#include "gxf/std/clock.hpp"
#include "gxf/std/gems/event_list/event_list.hpp"
#include "gxf/std/scheduler.hpp"
#include "gxf/std/scheduling_condition.hpp"

namespace nvidia {
namespace gxf {

// Forward declarations
class EntityExecutor;

/// @brief A basic single-threaded scheduler which tests scheduling term greedily
///
/// This scheduler is great for simple use cases and predictable execution. It evaluates
/// scheduling terms greedily and may incur a large overhead of scheduling term execution. Thus it
/// may not be suitable for large applications.
///
/// The scheduler requires a Clock to keep track of time. Based on the choice of clock the scheduler
/// will execute differently. If a Realtime clock is used the scheduler will execute in realtime.
/// This means for example pausing execution, i.e. sleeping the thread, until periodic scheduling
/// terms are due again. If a ManualClock is used scheduling will happen "time-compressed". This
/// means flow of time is altered to execute codelets immediately after each other.
class GreedyScheduler : public Scheduler {
 public:
  gxf_result_t registerInterface(Registrar* registrar) override;
  gxf_result_t initialize() override;
  gxf_result_t deinitialize() override;
  gxf_result_t prepare_abi(EntityExecutor* executor) override;
  gxf_result_t schedule_abi(gxf_uid_t eid) override;
  gxf_result_t unschedule_abi(gxf_uid_t eid) override;
  gxf_result_t runAsync_abi() override;
  gxf_result_t stop_abi() override;
  gxf_result_t wait_abi() override;
  gxf_result_t event_notify_abi(gxf_uid_t eid, gxf_event_t event) override;

 private:
  Parameter<Handle<Clock>> clock_;
  Parameter<bool> realtime_;
  Parameter<int64_t> max_duration_ms_;
  Parameter<bool> stop_on_deadlock_;
  Parameter<double> check_recession_period_ms_;
  Parameter<int64_t> stop_on_deadlock_timeout_;

  EntityExecutor* executor_ = nullptr;

  std::atomic_bool stopping_{true};
  std::atomic<gxf_result_t> thread_error_code_;
  std::unique_ptr<std::thread> thread_ = nullptr;

  // Used temporarily until realtime flag is removed.
  Entity clock_entity_;

  // Used for keeping track of async events
  uint64_t count_wait_event_{0};
  std::mutex event_mutex_;
  std::unique_ptr<EventList<gxf_uid_t>> event_notified_;
  std::unique_ptr<EventList<gxf_uid_t>> event_waiting_;
  std::condition_variable event_notification_cv_;

  // Used for keeping track of graph entities
  FixedVector<gxf_uid_t> active_entities_;
  FixedVector<gxf_uid_t> new_entities_;
  std::unique_ptr<EventList<gxf_uid_t>> unschedule_entities_;
  std::mutex entity_mutex_;
  // Dedicated mutex for each entity
  std::unordered_map<gxf_uid_t, std::unique_ptr<std::mutex>> entity_mutex_map_;

  // latest timestamp from last should_stop == false
  int64_t last_no_stop_ts_ = 0;
  // maintain last no stop timestamp, and check if need to update should_stop
  gxf_result_t stop_on_deadlock_timeout(const int64_t timeout, const int64_t now,
    bool& should_stop);
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_STD_GREEDY_SCHEDULER_HPP_
