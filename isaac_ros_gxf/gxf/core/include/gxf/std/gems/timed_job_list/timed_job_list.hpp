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

#ifndef NVIDIA_GXF_STD_GEMS_TIMED_JOB_LIST_TIMED_JOB_LIST_HPP_
#define NVIDIA_GXF_STD_GEMS_TIMED_JOB_LIST_TIMED_JOB_LIST_HPP_

#include <algorithm>
#include <condition_variable>
#include <list>
#include <mutex>
#include <queue>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "gxf/core/expected.hpp"

namespace nvidia {
namespace gxf {

// A thread-safe queue which sorts by target execution time. It provides a blocking pop to get
// jobs precisely when the time is right.
//  T: The type of jobs to store in the queue
//  Eq: (T,T) -> bool: true if jobs are identical
template <typename JobT>
class TimedJobList {
 public:
  using Clock_t = std::function<int64_t()>;

  TimedJobList(Clock_t clock) : clock_(clock), is_running_(false) {}
  // Number of queued jobs. This should only be used for logging purposes as it is not acquiring
  // a lock.
  size_t sizeUnsafe() const { return queue_.size(); }

  // thread-safe implementation to find number of queued jobs
  size_t size() const {
    std::unique_lock<std::mutex> lock(queue_cv_mutex_);
    return items_.size();
  }

  // Adds a job to the queue to be executed at the given target time. O(log(n))
  bool insert(JobT job, int64_t target_time, int64_t slack, int priority);
  // Notify that a job has completed.
  void notifyDone(JobT job);
  // This is a blocking call which will wait for an job.
  void waitForJob(JobT&);
  // Sets the job list to a running state
  void start() { is_running_.store(true); }
  // Sets the job list to a stopped state and wakes all waiting threads to flush.
  void stop() {
    // Acquire the lock to ensure a thread isn't trying to acquire a job
    // while invoking stop.
    std::unique_lock<std::mutex> lock(queue_cv_mutex_);
    // Stop the queue
    is_running_.store(false);
    // Wake all waiting threads
    queue_cv_.notify_all();
  }
  // Gets the target time of the next job; or returns an error if there is no job
  Expected<int64_t> getNextTargetTime() const {
    std::unique_lock<std::mutex> lock(queue_cv_mutex_);
    if (queue_.empty()) {
      return Unexpected{GXF_QUERY_NOT_FOUND};
    } else {
      return queue_.top().target_time;
    }
  }
  // wakes up one waiting threads
  void wakeOne() { queue_cv_.notify_one(); }
  // wakes all waiting threads
  void wakeAll() { queue_cv_.notify_all(); }

  // Pops the next pending or waiting job regardless of their
  // target execution time
  Expected<JobT> popFront() {
    std::unique_lock<std::mutex> lock(queue_cv_mutex_);

    // return pending jobs first
    for (auto pending_job = pending_.begin(); pending_job != pending_.end(); ++pending_job) {
      Item result = *pending_job;
      pending_.erase(pending_job);
      return result.job;
    }

    // return waiting jobs next
    if (queue_.empty()) { return Unexpected{GXF_FAILURE}; }

    Item result = queue_.top();
    queue_.pop();
    items_.erase(result.job);
    return result.job;
  }

  // Checks if there any pending or waiting jobs
  bool empty() const {
    std::unique_lock<std::mutex> lock(queue_cv_mutex_);
    return queue_.empty() && pending_.empty();
  }

 private:
  // We allow a small tolerance when executing jobs
// static constexpr int64_t kTimeFudge = 100'000;  // 0.1 ms
  static constexpr int64_t kTimeFudge = 1;  // 100 ns

  // Helper class used to store jobs with additional information
  struct Item {
    // The job to execute
    JobT job;
    // The target time at which the job should be executed
    int64_t target_time;
    // The amount of time the target time can slip by
    int64_t slack;
    // The priority is used as a tie breaker when two jobs would be scheduled close together.
    int priority;
  };

  // Sorts jobs such that the earliest non-running job has highest priority
  struct ItemPriorityCmp {
    bool operator()(const Item& a, const Item& b) {
      // If two jobs with their slack times are scheduled within a small
      // interval of each other check their priorities,
      // Otherwise earliest deadline goes first.
      const int64_t a_time = a.target_time + a.slack;
      const int64_t b_time = b.target_time + b.slack;

      if (std::abs(a_time - b_time) < kTimeFudge && a.priority != b.priority) {
        return a.priority < b.priority;
      } else {
        return a_time > b_time;
      }
    }
  };

  // This function pointer should be set to utilize the desired clock for the queue.
  Clock_t clock_;
  // state variable to control if threads should block on waiting for jobs
  std::atomic<bool> is_running_;

  mutable std::mutex queue_cv_mutex_;
  std::condition_variable queue_cv_;

  // List of future events sorted by their scheduled execution time.
  std::priority_queue<Item, std::vector<Item>, ItemPriorityCmp> queue_;

  // Item lookup to avoid duplicates
  std::unordered_set<JobT> items_;

  // List of events overdue, we have passed their scheduled execution time but have not been able to
  // run yet.
  std::list<Item> pending_;
};

// -------------------------------------------------------------------------------------------------

template <typename JobT>
bool TimedJobList<JobT>::insert(JobT job, int64_t target_time, int64_t slack, int priority) {
  {
    std::unique_lock<std::mutex> lock(queue_cv_mutex_);
    if (items_.find(job) != items_.end()) { return false; }
    items_.insert(job);
    queue_.push(Item{std::move(job), target_time, slack, priority});
  }
  queue_cv_.notify_one();
  return true;
}

template <typename JobT>
void TimedJobList<JobT>::notifyDone(JobT job) {
  std::unique_lock<std::mutex> lock(queue_cv_mutex_);
  wakeOne();
}

template <typename JobT>
void TimedJobList<JobT>::waitForJob(JobT& job) {
  while (is_running_) {
    // Acquire the lock to check for a job
    std::unique_lock<std::mutex> lock(queue_cv_mutex_);
    // Double check the running status as a stop could have been invoked.
    if (!is_running_) {
      return;
    }
    int64_t wait_duration = 0;
    const int64_t now = clock_();
    while (!queue_.empty()) {
      // We have a job, check if it is time for it
      Item top_item = queue_.top();
      const int64_t eta = top_item.target_time - now;
      // If the jobs target time is within our fudge factor fire now.
      if (eta <= kTimeFudge) {
        pending_.push_back(top_item);
        queue_.pop();
      } else {
        // wait till next eta minus a small window
        wait_duration = std::max(eta - kTimeFudge, static_cast<int64_t>(0));
        break;
      }
    }
    // Loop through the list of pending jobs to find one that can be run.
    for (auto pending_job = pending_.begin(); pending_job != pending_.end(); ++pending_job) {
      job = pending_job->job;
      pending_job->target_time = now;
      pending_.erase(pending_job);
      items_.erase(job);
      return;
    }
    // Wait for a new job or until time is up (the lock is released while waiting)
    if (wait_duration > 0) {
      queue_cv_.wait_for(lock, std::chrono::nanoseconds(wait_duration));
    } else {
      queue_cv_.wait(lock);
    }
  }
}

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_STD_GEMS_TIMED_JOB_LIST_TIMED_JOB_LIST_HPP_
