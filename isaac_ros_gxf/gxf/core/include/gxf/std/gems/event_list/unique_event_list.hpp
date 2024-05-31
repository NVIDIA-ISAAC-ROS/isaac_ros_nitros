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

#ifndef NVIDIA_GXF_UNIQUE_EVENT_LIST_HPP_
#define NVIDIA_GXF_UNIQUE_EVENT_LIST_HPP_

#include <algorithm>
#include <list>
#include <mutex>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "gxf/core/expected.hpp"
#include "gxf/core/gxf.h"

namespace nvidia {
namespace gxf {

// A simple thread safe list implementation to store and retrieve events
template <typename T>
class UniqueEventList {
 public:
  UniqueEventList() = default;
  UniqueEventList(const UniqueEventList<T>&) = delete;
  UniqueEventList& operator=(const UniqueEventList<T>&) = delete;

  // Adds an item on to the event list
  bool pushEvent(T item) {
    std::lock_guard<std::mutex> lock(list_mutex_);
    if (items_.find(item) != items_.end()) { return false; }
    list_.push_back(item);
    items_.insert(std::make_pair(item, --list_.end()));
    return true;
  }

  // Removes an event from the list if it exists and returns true,
  // returns false otherwise
  void removeEvent(T item) {
    std::lock_guard<std::mutex> lock(list_mutex_);

    if (items_.find(item) == items_.end()) { return; }
    list_.erase(items_.at(item));
    items_.erase(item);
  }

  // pops the first element in the list
  Expected<T> popEvent() {
    std::lock_guard<std::mutex> lock(list_mutex_);
    if (list_.empty()) { return Unexpected{GXF_FAILURE}; }

    T event = list_.front();
    list_.pop_front();
    items_.erase(event);
    return event;
  }

  // exports a copy of the events in the list
  std::list<T> exportList() const {
    std::lock_guard<std::mutex> lock(list_mutex_);
    return list_;
  }

  // checks if the list is empty
  bool empty() const {
    std::lock_guard<std::mutex> lock(list_mutex_);
    return list_.empty();
  }

  // returns size of the event list
  size_t size() const {
    std::lock_guard<std::mutex> lock(list_mutex_);
    return items_.size();
  }

  // checks if the list has event
  bool hasEvent(T item) const {
    std::lock_guard<std::mutex> lock(list_mutex_);
    return items_.find(item) != items_.end() ? true : false;
  }

  // clears the event list
  void clear() {
    std::lock_guard<std::mutex> lock(list_mutex_);
    list_.clear();
    items_.clear();
  }

  mutable std::mutex list_mutex_;
  std::list<T> list_;
  std::unordered_map<T, typename std::list<T>::iterator> items_;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_UNIQUE_EVENT_LIST_HPP_
