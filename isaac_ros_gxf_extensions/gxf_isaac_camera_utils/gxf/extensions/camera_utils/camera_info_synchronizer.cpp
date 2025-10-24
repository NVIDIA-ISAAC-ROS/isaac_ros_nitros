// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#include "camera_info_synchronizer.hpp"

#include <algorithm>
#include <limits>
#include <vector>

#include "gxf/core/expected_macro.hpp"
#include "gxf/std/timestamp.hpp"

namespace nvidia {
namespace isaac {

namespace {
// Global constants that detail sync behavior
size_t kSyncOldestOnFastestQueue = 0;
size_t kSyncNewestOnSlowestQueue = 1;
}  // namespace

gxf_result_t
CameraInfoSynchronization::registerInterface(gxf::Registrar* registrar) {
  gxf::Expected<void> result;
  result &= registrar->parameter(camera_message_rx_, "camera_message_rx",
                                 "Camera Message receiver",
                                 "Camera message receiver handle.");
  result &= registrar->parameter(
      camera_info_message_rx_, "camera_info_message_rx",
      "Camera Info message receiver", "Camera info message receiver handle.");
  result &= registrar->parameter(camera_message_tx_, "camera_message_tx",
                                 "Camera Message transmitter",
                                 "Camera message transmitter handle.");
  result &= registrar->parameter(camera_info_message_tx_, "camera_info_message_tx",
                           "Camera Info message transmitter",
                           "Camera info message transmitter handle.");
  result &= registrar->parameter(use_latest_camera_info_, "use_latest_camera_info",
                           "Use latest camera info",
                           "Use latest camera info message, make time stamp "
                           "same as image and forward",
                           false);
  result &= registrar->parameter(drop_old_messages_, "drop_old_messages",
                           "Drop oldest messages inside tick function",
                           "Drop oldest messages inside tick function", false);
  result &= registrar->parameter(
      sync_policy_, "sync_policy", "Synchronization policy",
      "Synchronization policy, 0: choose oldest message from fastest moving queue"
      "1: choose newest message from slowest moving queue. 0 is good for processing most of the"
      "data at the cost of being potentially slower than realtime. 1 is good for finding sync"
      "points close to real time but at the cost of dropping more data.",
      static_cast<uint64_t>(0));

  return gxf::ToResultCode(result);
}

gxf_result_t CameraInfoSynchronization::start() {
  camera_acq_times_.reserve(camera_message_rx_->capacity());
  camera_info_acq_times_.reserve(camera_info_message_rx_->capacity());
  return GXF_SUCCESS;
}

gxf_result_t CameraInfoSynchronization::dropOldestMessagesIfQueueIsFull() {
  // Clear all queues
  RETURN_IF_ERROR(camera_message_rx_->sync());
  if (camera_message_rx_->size() == camera_message_rx_->capacity()) {
    // If you see this log, please disable the drop_old_messages parameter and make sure your
    // GXF node is the first entry point into the Nitros graph. The Nitros graph will force add
    // a policy 0 on all incoming receivers and will drop messages if the queue is full.
    // This log is to inform the user that they might be unintentionally dropping messages.
    GXF_LOG_WARNING("Dropping messages from camera message receiver");
    RETURN_IF_ERROR(camera_message_rx_->receive());
  }
  RETURN_IF_ERROR(camera_info_message_rx_->sync());
  if (camera_info_message_rx_->size() == camera_info_message_rx_->capacity()) {
    // If you see this log, please disable the drop_old_messages parameter and make sure your
    // GXF node is the first entry point into the Nitros graph. The Nitros graph will force add
    // a policy 0 on all incoming receivers and will drop messages if the queue is full.
    // This log is to inform the user that they might be unintentionally dropping messages.
    GXF_LOG_WARNING("Dropping messages from camera info message receiver");
    RETURN_IF_ERROR(camera_info_message_rx_->receive());
  }
  return GXF_SUCCESS;
}

gxf_result_t CameraInfoSynchronization::useLatestCameraInfo() {
  bool all_messages_available = true;

  RETURN_IF_ERROR(camera_message_rx_->sync());
  if (camera_message_rx_->size() == 0) {
    GXF_LOG_DEBUG(
        "Camera message receiver has no messages for synchronization!");
    all_messages_available = false;
  }

  RETURN_IF_ERROR(camera_info_message_rx_->sync());
  if (camera_info_message_rx_->size() == 0) {
    GXF_LOG_DEBUG(
        "Camera info message receiver has no messages for synchronization!");
    all_messages_available = false;
  }

  if (!all_messages_available && drop_old_messages_.get()) {
    // We do this to keep the graph ticking and not leading to blocks.
    // To enable no dropping of messages, users are requested to use individual
    // MessageAvailableSchedulingTerms for each rx. We do not need to do this if used as the first
    // input in a Nitros node because Nitros adds a policy: 0 to the input rx of the graph
    // automatically.
    return dropOldestMessagesIfQueueIsFull();
  } else if (!all_messages_available) {
    // Do not do anything if not all messages are available
    return GXF_SUCCESS;
  }

  gxf::Entity camera_message = UNWRAP_OR_RETURN(camera_message_rx_->receive());
  gxf::Entity camera_info_message = UNWRAP_OR_RETURN(camera_info_message_rx_->receive());

  // Make camera info timestamp same as camera message
  auto timestamp_components_cam = camera_message.findAllHeap<gxf::Timestamp>();
  auto acq_time = timestamp_components_cam->front().value()->acqtime;

  // This will publish the camera and camera info messages with the same
  // timestamp. This might be required upstream by some codelets
  camera_message_tx_->publish(camera_message, acq_time);
  camera_info_message_tx_->publish(camera_info_message, acq_time);

  return GXF_SUCCESS;
}

gxf_result_t syncMessages(const std::vector<int64_t>& acq_times,
                          const gxf::Handle<gxf::Receiver>& rx,
                          const gxf::Handle<gxf::Transmitter>& tx,
                          const int64_t candidate_sync_point) {
  for (size_t index = acq_times.size(); index > 0; index--) {
    if (acq_times[index-1] == candidate_sync_point) {
      gxf::Entity message = UNWRAP_OR_RETURN(rx->receive());
      RETURN_IF_ERROR(tx->publish(message));
      return GXF_SUCCESS;
    } else if (acq_times[index-1] < candidate_sync_point) {
      // Old message, drop it
      RETURN_IF_ERROR(rx->receive());
    } else if (acq_times[index-1] > candidate_sync_point) {
      // This should not happen
      GXF_LOG_ERROR("Message timestamp is not monotonically increasing");
      return GXF_FAILURE;
    }
  }
  GXF_LOG_ERROR("Did not publish anything, should have published");
  return GXF_FAILURE;
}

gxf_result_t CameraInfoSynchronization::tick() {
  // Clear all queues for processing in this tick call
  camera_acq_times_.clear();
  camera_info_acq_times_.clear();

  if (use_latest_camera_info_.get()) {
    return useLatestCameraInfo();
  }
  // Check if all input queues have messages
  bool all_messages_available = true;
  RETURN_IF_ERROR(camera_message_rx_->sync());
  RETURN_IF_ERROR(camera_info_message_rx_->sync());
  if (camera_message_rx_->size() == 0 || camera_info_message_rx_->size() == 0) {
    // Not all the inputs have messages, unexpected.
    GXF_LOG_DEBUG("Not all the inputs have messages for synchronization!");
    all_messages_available = false;
  }

  if (!all_messages_available && drop_old_messages_.get()) {
    // Acts as a policy inside the tick function
    return dropOldestMessagesIfQueueIsFull();
  } else if (!all_messages_available) {
    // Do not do anything if not all messages are available
    return GXF_SUCCESS;
  }

  // Read the timestamps of available messages from camera input
  for (size_t index = camera_message_rx_->size(); index > 0; index--) {
    auto msg_result = camera_message_rx_->peek(index - 1);
    if (msg_result) {
      auto timestamp_components = msg_result->findAllHeap<gxf::Timestamp>();
      if (!timestamp_components) {
        return gxf::ToResultCode(timestamp_components);
      }
      if (0 == timestamp_components->size()) {
        GXF_LOG_ERROR("No timestamp found from the camera message");
        return GXF_ENTITY_COMPONENT_NOT_FOUND;
      }
      camera_acq_times_.push_back(
          timestamp_components->front().value()->acqtime);
    }
  }

  // Read the timestamps of available messages from camera info input
  for (size_t index = camera_info_message_rx_->size(); index > 0; index--) {
    auto msg_result = camera_info_message_rx_->peek(index - 1);
    if (msg_result) {
      auto timestamp_components = msg_result->findAllHeap<gxf::Timestamp>();
      if (!timestamp_components) {
        return gxf::ToResultCode(timestamp_components);
      }
      if (0 == timestamp_components->size()) {
        GXF_LOG_ERROR("No timestamp found from the camera info message");
        return GXF_ENTITY_COMPONENT_NOT_FOUND;
      }
      camera_info_acq_times_.push_back(
          timestamp_components->front().value()->acqtime);
    }
  }

  if (camera_acq_times_.size() == 0 || camera_info_acq_times_.size() == 0) {
    GXF_LOG_ERROR("This array should not be empty at this stage");
    return GXF_FAILURE;
  }

  int64_t candidate_sync_point = std::numeric_limits<int64_t>::max();
  if (sync_policy_.get() == kSyncNewestOnSlowestQueue) {
    /* Find  newest timestamp from slowest moving queue, this makes sure with
      the messages we have we will find a sync point closest to real time, and
      not the potential non optimal time chosen by oldest message in fastest
      moving queue
      Each timestamps.front() is the newest message from that queue.We pick the minimum of
      these newest timestamps
    */
    if (!camera_acq_times_.empty() && camera_acq_times_.front() < candidate_sync_point) {
      candidate_sync_point = camera_acq_times_.front();
    }
    if (!camera_info_acq_times_.empty() && camera_info_acq_times_.front() < candidate_sync_point) {
      candidate_sync_point = camera_info_acq_times_.front();
    }
  } else if (sync_policy_.get() == kSyncOldestOnFastestQueue) {
    // find oldest timestamp we're going to pick from the fastest moving queue
    candidate_sync_point = 0;
    if (camera_acq_times_.back() > candidate_sync_point) {
      candidate_sync_point = camera_acq_times_.back();
    }
    if (camera_info_acq_times_.back() > candidate_sync_point) {
      candidate_sync_point = camera_info_acq_times_.back();
    }
  }

  bool found_in_camera_queue = false;
  bool found_in_camera_info_queue = false;
  for (auto it = camera_acq_times_.begin(); it != camera_acq_times_.end(); it++) {
    if (*it == candidate_sync_point) {
      found_in_camera_queue = true;
    }
  }
  for (auto it = camera_info_acq_times_.begin(); it != camera_info_acq_times_.end(); it++) {
    if (*it == candidate_sync_point) {
      found_in_camera_info_queue = true;
    }
  }

  if (!found_in_camera_queue || !found_in_camera_info_queue) {
    GXF_LOG_DEBUG("Did not find candidate sync point in one of the input queues");
    return GXF_SUCCESS;
  }

  GXF_LOG_DEBUG("Candidate timestamp for syncing (from camera queue): %zd",
                candidate_sync_point);
  RETURN_IF_ERROR(syncMessages(camera_info_acq_times_, camera_info_message_rx_,
                               camera_info_message_tx_, candidate_sync_point));
  RETURN_IF_ERROR(syncMessages(camera_acq_times_, camera_message_rx_,
                               camera_message_tx_, candidate_sync_point));
  return GXF_SUCCESS;
}

}  // namespace isaac
}  // namespace nvidia
