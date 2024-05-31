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

#ifndef NVIDIA_GXF_STD_EVENT_BASED_SCHEDULER_HPP_
#define NVIDIA_GXF_STD_EVENT_BASED_SCHEDULER_HPP_

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <shared_mutex>  // NOLINT
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "common/fixed_map.hpp"
#include "gxf/core/entity.hpp"
#include "gxf/core/handle.hpp"
#include "gxf/std/clock.hpp"
#include "gxf/std/cpu_thread.hpp"
#include "gxf/std/gems/event_list/unique_event_list.hpp"
#include "gxf/std/gems/staging_queue/staging_queue.hpp"
#include "gxf/std/gems/timed_job_list/timed_job_list.hpp"
#include "gxf/std/resources.hpp"
#include "gxf/std/scheduler.hpp"
#include "gxf/std/scheduling_condition.hpp"

namespace nvidia {
namespace gxf {


// Forward declarations
class EntityExecutor;

/// @brief Event based scheduler
///
/// The scheduler requires a Clock to keep track of time. It spawns a dispatcher thread and several
/// worker threads as configured. Dispatcher thread checks condition of entities and place them in
/// worker queue if they are ready to run or have expected time to run.
class EventBasedScheduler : public Scheduler {
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
  gxf_result_t notifyDispatcher(gxf_uid_t eid = kNullUid);

 private:
  // Enum to indicate if a worker thread has acquired an entity for execution or not
  enum class EntityOwnership : uint8_t {
    kFree = 0,
    kAcquired
  };

  class ScheduleEntity {
   public:
    ScheduleEntity(gxf_uid_t eid, const char * name) {
      eid_ = eid;
      name_ = std::string(name);
      condition_ = SchedulingCondition{SchedulingConditionType::READY, 0};
      ownership_ = EntityOwnership::kFree;
      queue_index_ = -1;
      thread_pool_id_ = -1;
      thread_id_ = kDefaultThreadPoolThreadId;
      unschedule_ = false;
      is_present_in_ready_queue_ = false;
    }

    void updateCondition() {
      condition_ = SchedulingCondition{SchedulingConditionType::READY, 0};
    }

    Expected<void> tryToAcquire(void) {
      EntityOwnership free = EntityOwnership::kFree;
      if (ownership_.compare_exchange_strong(free, EntityOwnership::kAcquired)) {
        return Success;
      }
      return Unexpected{GXF_FAILURE};
    }

    Expected<void> releaseOwnership(void) {
      EntityOwnership acquired = EntityOwnership::kAcquired;
      if (ownership_.compare_exchange_strong(acquired, EntityOwnership::kFree)) {
        return Success;
      }
      return Unexpected{GXF_FAILURE};
    }

    int32_t queue_index_;
    gxf_uid_t eid_;
    SchedulingCondition condition_;
    std::string name_;
    std::atomic<EntityOwnership> ownership_;
    int32_t thread_pool_id_;
    gxf_uid_t thread_id_;
    bool unschedule_;
    std::shared_timed_mutex ready_queue_sync_mutex_;
    bool is_present_in_ready_queue_;
  };

  std::unordered_map<gxf_uid_t, std::shared_ptr<ScheduleEntity>> entities_;

  struct TimeStat {
    std::atomic<uint64_t> execTime = {0};
    std::atomic<uint64_t> waitTime = {0};
    std::atomic<uint64_t> execCount = {0};
  };

  struct TimeStat dispatcherStats;
  struct TimeStat workerStats;           // operations from multiple worker threads

  static constexpr int64_t kMsToNs = 1'000'000l;      // Convenient constant of 1 ms = 1e6 ns
  static constexpr int64_t kMaxSlipNs = 1 * kMsToNs;  // Max slip tolerance set to 1 ms
  static constexpr size_t kMaxThreads = 128;          // Max number of worker threads

  // Common thread id for all the worker threads in default thread pool
  static constexpr int64_t kDefaultThreadPoolThreadId = -1;
  // Position of ready queue for threads belonging to default thread pool
  static constexpr int64_t kDefaultThreadPoolQueueIndex = 0;
  // Entrance for dispatcher threads
  void dispatcherThreadEntrance();
  // notify the dispatcher thread
  // Move entity to some queue
  void dispatchEntity(std::shared_ptr<ScheduleEntity> e);
  void dispatchEntityAsync(std::shared_ptr<ScheduleEntity> e);
  // Entrance for async event handler thread
  void asyncEventThreadEntrance();
  // Entrance for worker threads
  void workerThreadEntrance(ThreadPool* pool, int64_t thread_number);

  // Checks if need to stop due to no active entities or expiration
  bool checkEndingCriteria(int64_t timestamp);

  // Updates condition of specific entity id and keep track of how many entities are good to run
  void updateCondition(std::shared_ptr<ScheduleEntity> e,
   const SchedulingCondition& next_condition);

  // stops all async threads and deactivates all the entities
  gxf_result_t stopAllThreads();

  // stops all jobs queued in dispatcher and workers
  void stopAllJobs();

  // returns the current state of the scheduler as string
  const char* schedulerStateString();
  // cache thread info for pinned job into resources_
  void prepareResourceMapStrict(std::shared_ptr<ScheduleEntity> e);
  // index of the ready queue in the ready_wait_time_jobs_ vector where entity is supposed to go

  uint64_t getReadyCount();

  // Parameters
  Parameter<Handle<Clock>> clock_;
  Parameter<int64_t> max_duration_ms_;
  Parameter<bool> stop_on_deadlock_;
  Parameter<int64_t> stop_on_deadlock_timeout_;
  Parameter<int64_t> worker_thread_number_;
  Parameter<bool> thread_pool_allocation_auto_;

  EntityExecutor* executor_ = nullptr;

  ThreadPool default_thread_pool_;

  // thread pool set including default pool and added pools
  std::set<ThreadPool*> thread_pool_set_;

  // map for storing the relevant ready_wait_queue
  FixedMap<gxf_uid_t, int> thread_queue_mapping_;

  gxf_result_t thread_error_code_;

  // A thread to dispatch jobs to worker pool
  std::thread dispatcher_thread_;

  // A thread to trigger exit after max time duration
  std::thread max_duration_thread_;

  // Mutex to synchronize dispatcher thread
  std::mutex dispatcher_sync_mutex_;

  // Mutex to synchronize max duration thread
  std::mutex max_duration_sync_mutex_;

  // instances of dispatcher/worker threads. 0 index would be async event handler thread.
  std::vector<std::thread> async_threads_;

  // Keep track of the timestamp that the scheduler started working for checking expiration
  int64_t start_timestamp_ = 0;

  // Queue with execution time for jobs to execute
  std::vector<std::unique_ptr<TimedJobList<gxf_uid_t>>> ready_wait_time_jobs_;

  // synchronize entities
  std::mutex external_event_notification_mutex_;
  std::mutex internal_event_notification_mutex_;

  // Mutex to synchronize worker thread pool
  std::mutex thread_sync_mutex_;
  std::condition_variable thread_sync_cv_;

  // Keep track of asynchronous external events
  std::unique_ptr<UniqueEventList<gxf_uid_t>> external_event_notified_;
  // Holds entities waiting for some event
  std::unique_ptr<UniqueEventList<gxf_uid_t>> event_waiting_;
  // Holds entities waiting for custom conditions
  std::unique_ptr<UniqueEventList<gxf_uid_t>> waiting_;

  // Keep track of asynchronous internal events
  std::unique_ptr<UniqueEventList<gxf_uid_t>> internal_event_notified_;

  // Running count of entities in flight / execution
  std::atomic<uint8_t> running_threads_{0};

  std::condition_variable external_event_notification_cv_;
  std::condition_variable internal_event_notification_cv_;

  std::condition_variable max_duration_thread_cv_;

  // enum type to represent scheduler state changes
  enum class State : uint8_t {
    kNotStarted = 0,       // Scheduler has not started execution yet, starts during runAsync_abi()
    kRunning,              // Execution threads are running
    kStopping,             // Ending criteria reached, execution threads are doing pending jobs
    kStopped,              // Scheduler has stopped
  };

  std::atomic<State> state_{State::kNotStarted};
  std::mutex state_change_mutex_;
  std::condition_variable work_done_cv_;

  // latest timestamp from last should_stop == false
  int64_t last_no_stop_ts_ = 0;
  // maintain last no stop timestamp, and check if need to update should_stop
  gxf_result_t stop_on_deadlock_timeout(const int64_t timeout,
    const int64_t now, bool& should_stop);
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_STD_EVENT_BASED_SCHEDULER_HPP_
