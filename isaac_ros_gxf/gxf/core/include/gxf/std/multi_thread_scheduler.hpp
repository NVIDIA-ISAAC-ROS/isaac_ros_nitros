// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2021-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef NVIDIA_GXF_STD_MUTLI_THREAD_SCHEDULER_HPP_
#define NVIDIA_GXF_STD_MUTLI_THREAD_SCHEDULER_HPP_

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "gxf/core/entity.hpp"
#include "gxf/core/handle.hpp"
#include "gxf/std/clock.hpp"
#include "gxf/std/cpu_thread.hpp"
#include "gxf/std/gems/event_list/event_list.hpp"
#include "gxf/std/gems/staging_queue/staging_queue.hpp"
#include "gxf/std/gems/timed_job_list/timed_job_list.hpp"
#include "gxf/std/scheduler.hpp"
#include "gxf/std/scheduling_condition.hpp"

namespace nvidia {
namespace gxf {

// Forward declarations
class EntityExecutor;

constexpr int64_t kMsToNs = 1'000'000l;      // Convenient constant of 1 ms = 1e6 ns
constexpr int64_t kMaxSlipNs = 1 * kMsToNs;  // Max slip tolerance set to 1 ms

/// @brief Multi-threaded scheduler
///
/// The scheduler requires a Clock to keep track of time. It spawns a dispatcher thread and several
/// worker threads as configured. Dispatcher thread checks condition of entities and place them in
/// worker queue if they are ready to run or have expected time to run.
class MultiThreadScheduler : public Scheduler {
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
  std::atomic<uint64_t> workerExecTime { 0 };
  std::atomic<uint64_t> workerWaitTime { 0};
  std::atomic<uint64_t> workerExecCount { 0 };
  uint64_t dispatcherExecTime = 0;
  uint64_t dispatcherWaitTime = 0;
  uint64_t dispatcherExecCount = 0;
  // Entrance for dispatcher threads
  void dispatcherThreadEntrance();
  // Entrance for async event handler thread
  void asyncEventThreadEntrance();
  // Entrance for worker threads
  void workerThreadEntrance(ThreadPool* pool, int64_t thread_number);

  // Checks if need to stop due to no active entities or expiration
  void checkEndingCriteria(int64_t timestamp);

  // Updates condition of specific entity id and keep track of how many entities are good to run
  void updateCondition(gxf_uid_t eid, const SchedulingCondition& next_condition);

  // stops all async threads and deactivates all the entities
  gxf_result_t stopAllThreads();

  // stops all jobs queued in dispatcher and workers
  void stopAllJobs();

  // returns the current state of the scheduler as string
  const char* schedulerStateString();

  // cache thread info for pinned job into resources_
  void prepareResourceMap(gxf_uid_t eid);
  // cache thread info for every job into resources_, use default thread pool for non-pinned job
  void prepareResourceMapStrict(gxf_uid_t eid);
  // In worker thread, check if pinned job matches the thread
  bool isJobMatch(ThreadPool* pool, int64_t thread_number, gxf_uid_t eid);
  // In worker thread, check if the job matches the thread and if thread matches the job
  bool isJobMatchStrict(ThreadPool* pool, int64_t thread_number, gxf_uid_t eid);

  // Parameters
  Parameter<Handle<Clock>> clock_;
  Parameter<int64_t> max_duration_ms_;
  Parameter<double> check_recession_period_ms_;
  Parameter<bool> stop_on_deadlock_;
  Parameter<int64_t> stop_on_deadlock_timeout_;
  Parameter<int64_t> worker_thread_number_;
  Parameter<bool> thread_pool_allocation_auto_;
  Parameter<bool> strict_job_thread_pinning_;

  EntityExecutor* executor_ = nullptr;

  ThreadPool default_thread_pool_;

  // thread pool set including default pool and added pools
  std::set<ThreadPool*> thread_pool_set_;

  // Resource map for thread pools and worker threads
  std::map<gxf_uid_t, std::pair<ThreadPool*, int64_t>> resources_;

  gxf_result_t thread_error_code_;

  // A thread to dispatch jobs to worker pool
  std::thread dispatcher_thread_;

  // Mutex to synchronize dispatcher thread
  std::mutex dispatcher_sync_mutex_;

  // instances of dispatcher/worker threads. 0 index would be async event handler thread.
  std::vector<std::thread> async_threads_;

  // Keep track of the timestamp that the scheduler started working for checking expiration
  int64_t start_timestamp_ = 0;

  // Keep track of conditions for entities
  std::unordered_map<gxf_uid_t, SchedulingCondition> conditions_;
  int64_t ready_count_{0};
  int64_t wait_time_count_{0};
  int64_t wait_event_count_{0};
  int64_t wait_count_{0};
  std::mutex conditions_mutex_;

  // Queue with execution time for jobs to execute
  std::unique_ptr<TimedJobList<gxf_uid_t>> worker_jobs_;

  // Queue worker threads => dispatcher thread
  std::unique_ptr<TimedJobList<gxf_uid_t>> check_jobs_;

  // synchronize entities
  std::mutex event_notification_mutex_;
  std::unique_ptr<EventList<gxf_uid_t>> unschedule_entities_;

  // Mutex to synchronize worker thread pool
  std::mutex thread_sync_mutex_;
  std::condition_variable thread_sync_cv_;

  // Keep track of asynchronous events
  std::unique_ptr<EventList<gxf_uid_t>> event_notified_;
  std::unique_ptr<EventList<gxf_uid_t>> event_waiting_;
  std::condition_variable event_notification_cv_;

  enum class State {
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

#endif  // NVIDIA_GXF_STD_MUTLI_THREAD_SCHEDULER_HPP_
