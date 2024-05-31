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

#ifndef NVIDIA_GXF_STD_GEMS_QUEUE_THREAD_HPP
#define NVIDIA_GXF_STD_GEMS_QUEUE_THREAD_HPP

#include <sys/syscall.h>
#include <unistd.h>

#include <condition_variable>
#include <functional>
#include <future>
#include <list>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

#include "common/assert.hpp"
#include "common/logger.hpp"

#define STR(str) ((str).empty() ? "DefaultName" : (str).c_str())
namespace nvidia {
namespace gxf {

/**
 * Entry wrapper for the user item and its associated promise
*/
template <typename ItemType>
struct UserItemWithPromise {
  // typedef ItemType user_item_type;
  ItemType user_item;
  std::promise<bool> promise;
  bool is_stop_signal = false;

  UserItemWithPromise(ItemType i) : user_item(std::move(i)) {}
  UserItemWithPromise() : is_stop_signal(true) {}
};

/**
 * A thread safe generic container serving as the queue for QueueThread
*/
template <typename ItemType>
class GuardQueue {
 public:
using Entry = UserItemWithPromise<ItemType>;
  void push(Entry data) {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    queue_.emplace_back(std::move(data));
    queue_cv_.notify_one();
  }
  Entry pop() {
    GXF_LOG_VERBOSE("GuardQueue::pop() acquiring lock...");
    std::unique_lock<std::mutex> lock(queue_mutex_);
    GXF_LOG_VERBOSE("GuardQueue::pop() acquired lock, waiting on cv...");
    queue_cv_.wait(
      lock, [this]() {
        bool wait_condition = wakeup_once_ || !queue_.empty();
        GXF_LOG_VERBOSE("GuardQueue::pop() cv wait condition[%d]", wait_condition);
        return wait_condition; });
    if (wakeup_once_) {
      wakeup_once_ = false;
      GXF_LOG_VERBOSE("GuardQueue pop end on wakeup signal");
      // empty Entry carries is_stop_signal true
      return Entry();
    }
    GXF_ASSERT_TRUE(!queue_.empty());
    Entry ret = std::move(*queue_.begin());
    queue_.erase(queue_.begin());
    GXF_LOG_VERBOSE("GuardQueue::pop() popped an entry");
    return ret;
  }
  void wakeupOnce() {
    GXF_LOG_VERBOSE("GuardQueue trigger wakeup once");
    std::unique_lock<std::mutex> lock(queue_mutex_);
    wakeup_once_ = true;
    queue_cv_.notify_all();
    GXF_LOG_VERBOSE("GuardQueue finish wakeup once notification");
  }
  void clear() {
    GXF_LOG_VERBOSE("GuardQueue clear");
    std::unique_lock<std::mutex> lock(queue_mutex_);
    queue_.clear();
    wakeup_once_ = false;
  }
  int size() {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    return queue_.size();
  }

 private:
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::list<Entry> queue_;
  bool wakeup_once_ = false;
};

/**
 * The Async Runnable
 * Usage is similar to std::thread, but here the run function is event-driven callback.
 *
 * Construction:
 * 1. user event type in template;
 * 2. user run function implementation that takes user event type;
 * 3. thread name.
 *
 * Upon startup or no event is enqueued, the process thread remains sleep.
 * Whenever an event is raised in caller thread, simply enqueue that event
 * then the run function callback will process the event asynchronously
 *
 * Optionally an std::future is returned for each enqueued event, such that
 * users can synchronize completion of the event.
*/
template <typename ItemType>
class QueueThread {
 public:
  using Entry = UserItemWithPromise<ItemType>;
  using RunFunction = std::function<bool(ItemType)>;

  /**
   * Constructor, asynchronously running RunFunction
  */
  QueueThread(RunFunction run_function, const std::string& name)
    : run_function_(run_function) {
    std::promise<void> promise;
    std::future<void> future = promise.get_future();
    GXF_LOG_DEBUG("QueueThread starting new thread");
    thread_ = std::thread([&promise, this]() {
      GXF_LOG_DEBUG("QueueThread std::thread created[name: %s, ID: %s]",
        this->name_.c_str(), callerThreadId().c_str());
      promise.set_value();
      this->threadLoop();
    });
    setThreadName(name);
    future.wait();
  }

  /**
   * set thread name for profiling display
  */
  void setThreadName(const std::string& name) {
    GXF_ASSERT_TRUE(!name.empty());
    name_ = name;
    // POSIX pthread_setname_np max length limit 15 characters plus null terminator
    std::string pthread_name = (name.length() > 15) ? name.substr(0, 15) : name;
    if (thread_.joinable()) {
      if (pthread_setname_np(thread_.native_handle(), pthread_name.c_str()) != 0) {
        GXF_LOG_ERROR("set thread name: %s failed", STR(pthread_name));
        return;
      }
      GXF_LOG_DEBUG("QueueThread set new thread name: %s", STR(pthread_name));
    }
  }

  /**
   * Destructor
  */
  ~QueueThread() {
    if (!joined_) {
      // Ensure the stop logic is called if not already done so
      stop();
    }
    // Clear any remaining items after the thread has been joined
    guard_queue_.clear();
  }

  /**
   * Main API. Enqueue item to process, return future to sync the result.
  */
  std::future<bool> queueItem(ItemType item) {
    Entry itemWithPromise(std::move(item));
    std::future<bool> future = itemWithPromise.promise.get_future();
    guard_queue_.push(std::move(itemWithPromise));

    return future;
  }

  /**
   * Thread safe size check
  */
  int size() {
    return guard_queue_.size();
  }

  /**
   * Stop the Async Runnable
  */
  void stop() {
    std::string caller_thread_id = callerThreadId();
    GXF_LOG_DEBUG("QueueThread[%s]::stop() caller thread[%s] acquiring stop lock...",
      name_.c_str(), caller_thread_id.c_str());
    {
      std::lock_guard<std::mutex> lock(stop_mutex_);
      stop_requested_ = true;
    }
    GXF_LOG_DEBUG("QueueThread[%s]::stop() caller thread[%s] acquired stop lock",
      name_.c_str(), caller_thread_id.c_str());
    // Wake up the thread if it's waiting for new items
    guard_queue_.wakeupOnce();
    // Notify the wait() that stop was requested
    stop_cv_.notify_all();
    // Try join the thread, thread safe
    joinThread();
  }

  /**
   * Wait the Async Runnable
  */
  void wait() {
    std::string caller_thread_id = callerThreadId();
    GXF_LOG_DEBUG("QueueThread[%s]::wait() caller thread[%s] acquiring stop lock...",
      name_.c_str(), caller_thread_id.c_str());

    std::unique_lock<std::mutex> lock(stop_mutex_);

    GXF_LOG_DEBUG("QueueThread[%s]::wait() caller thread[%s] acquired stop lock",
      name_.c_str(), caller_thread_id.c_str());
    // Block caller thread
    stop_cv_.wait(lock, [this]() {
      bool wait_condition = stop_requested_ && guard_queue_.size() == 0;
      GXF_LOG_DEBUG("stop_requested_[%d] && guard_queue_.size()[%d], cv wait condition[%d]",
        stop_requested_.load(), guard_queue_.size(), wait_condition);
      return wait_condition; });
    // Try join the thread, thread safe
    joinThread();
  }

 private:
  /**
   * Thread safe join thread
  */
  void joinThread() {
    std::string caller_thread_id = callerThreadId();
    GXF_LOG_DEBUG("QueueThread[%s]::joinThread() caller thread[%s] acquiring join lock...",
      name_.c_str(), caller_thread_id.c_str());

    // Ensure only one caller can join the thread
    std::lock_guard<std::mutex> lock(join_mutex_);

    GXF_LOG_DEBUG("QueueThread[%s]::joinThread() caller thread[%s] acquired join lock",
      name_.c_str(), caller_thread_id.c_str());
    if (thread_.joinable()) {
      GXF_LOG_DEBUG("QueueThread[%s]::joinThread() got its thread joinable(), joining...",
        name_.c_str());
      thread_.join();
      GXF_LOG_DEBUG("QueueThread[%s]::joinThread() got its thread joined", name_.c_str());
      joined_ = true;
    }
  }

  /**
   * Thread loop listening and processing tasks
  */
  void threadLoop() {
    while (true) {
      try {
        Entry entry = guard_queue_.pop();
        if (entry.is_stop_signal) {
          GXF_LOG_INFO("QueueThread[%s]::threadLoop() return", STR(name_));
          return;  // okay to return
        }
        if (stop_requested_ && guard_queue_.size() == 0) {
          // Exit loop if stop is requested and all entries are processed
          break;
        }
        bool result = run_function_(std::move(entry.user_item));
        entry.promise.set_value(result);
        if (!result) {
          GXF_LOG_INFO("QueueThread[%s]::threadLoop() return and stop", STR(name_));
          break;  // don't return
        }
      }
      catch (const std::exception& e) {  // unexpected but continue
        GXF_LOG_ERROR("QueueThread:%s %s", STR(name_), e.what());
        // move on to next
        continue;
      }
      catch (...) {  // unexpected
        GXF_LOG_ERROR(
          "QueueThread:%s internal unexpected error, may cause stop", STR(name_));
        // Usually can move on to next, but need to check
        continue;
      }
    }

    GXF_LOG_DEBUG("threadLoop() break while loop");
    // Clear the queue before exiting the thread loop
    guard_queue_.clear();
    // unblock external thread blocking on wait() call
    stop_requested_ = true;
    // Notify that the thread is about to exit after processing all entries
    stop_cv_.notify_all();
  }

 private:
  // basic member variables
  std::thread thread_;
  std::string name_;
  RunFunction run_function_;
  GuardQueue<ItemType> guard_queue_;

  // member variable for stopping
  std::atomic<bool> stop_requested_{false};
  std::mutex stop_mutex_;
  std::condition_variable stop_cv_;

  // member variable for joining thread
  std::mutex join_mutex_;
  bool joined_ = false;

 private:
  std::string callerThreadId() {
    pid_t tid = syscall(SYS_gettid);
    std::stringstream ss;
    ss << tid;
    return ss.str();
  }
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_STD_GEMS_QUEUE_THREAD_HPP
