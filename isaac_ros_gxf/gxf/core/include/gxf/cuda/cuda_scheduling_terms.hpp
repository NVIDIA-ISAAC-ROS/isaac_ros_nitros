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
#ifndef NVIDIA_GXF_CUDA_CUDA_SCHEDULING_TERMS_HPP_
#define NVIDIA_GXF_CUDA_CUDA_SCHEDULING_TERMS_HPP_

#include <cuda_runtime.h>

#include <atomic>
#include <map>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "gxf/core/parameter_registrar.hpp"
#include "gxf/cuda/cuda_common.hpp"
#include "gxf/cuda/cuda_event.hpp"
#include "gxf/cuda/cuda_stream_id.hpp"
#include "gxf/std/receiver.hpp"
#include "gxf/std/scheduling_term.hpp"

namespace nvidia {
namespace gxf {

/**
 * @brief Scheduling term to monitor activity on a cuda stream
 *
 * A component which specifies the availability of data at the receiver on completion of
 * the work on the provided cuda stream with the help of callback function to host.
 * This scheduling term will register a call back function which will be called once the work on the
 * specified cuda stream completes indicating that the data is available for consumption
 */
class CudaStreamSchedulingTerm : public SchedulingTerm {
 public:
  // state of the scheduling term
  enum class State : int8_t {
    UNSET = 0,                // Initial state not waiting for any cuda host callback
    CALLBACK_REGISTERED,      // Indicates a call back function is registered
    DATA_AVAILABLE,           // Indicates data is ready to be consumed
  };

  gxf_result_t registerInterface(Registrar* registrar) override;
  gxf_result_t check_abi(int64_t timestamp, SchedulingConditionType* type,
                         int64_t* target_timestamp) const override;
  gxf_result_t onExecute_abi(int64_t timestamp) override;
  gxf_result_t update_state_abi(int64_t timestamp) override;

 private:
  // Receiver to monitor cuda stream activity
  Parameter<Handle<Receiver>> receiver_;
  // The current state of the scheduling term
  SchedulingConditionType current_state_;
  // Current message entity being monitored
  std::atomic<gxf_uid_t> message_eid_ = kNullUid;
  // State of the scheduling term
  std::atomic<State> state_{State::UNSET};
  // cuda host callback function on completion of all the queued work
  static void CUDART_CB cudaHostCallback(void* term_ptr);
};

/**
 * @brief Scheduling term based on cuda event
 *
 * A component which specifies the availability of data at the receiver on completion of the
 * work on the provided cuda stream with the help of cuda event.
 * This scheduling term will keep polling on the event provided to check for data availability for
 * consumption.
 */
class CudaEventSchedulingTerm : public SchedulingTerm {
 public:
  gxf_result_t registerInterface(Registrar* registrar) override;
  gxf_result_t initialize() override;
  gxf_result_t check_abi(int64_t timestamp, SchedulingConditionType* type,
                         int64_t* target_timestamp) const override;
  gxf_result_t onExecute_abi(int64_t timestamp) override;
  gxf_result_t update_state_abi(int64_t timestamp) override;

 private:
  Parameter<Handle<Receiver>> receiver_;
  Parameter<std::string> event_name_{};
  SchedulingConditionType current_state_;   // The current state of the scheduling term
  int64_t last_state_change_;               // timestamp when the state changed the last time
};


/**
 * @brief Scheduling term based on data availability in a cuda buffer
 *
 * A component which specifies the availability of data at the receiver based on
 * the cuda buffers present in incoming messages.
 *
 */
class CudaBufferAvailableSchedulingTerm : public SchedulingTerm {
 public:
  // state of the scheduling term
  enum class State : int8_t {
    UNSET = 0,                // Initial state not waiting for any cuda host callback
    CALLBACK_REGISTERED,      // Indicates a call back function is registered
    DATA_AVAILABLE,           // Indicates data is ready to be consumed
  };
  gxf_result_t registerInterface(Registrar* registrar) override;
  gxf_result_t check_abi(int64_t timestamp, SchedulingConditionType* type,
                         int64_t* target_timestamp) const override;
  gxf_result_t onExecute_abi(int64_t timestamp) override;
  gxf_result_t update_state_abi(int64_t timestamp) override;

 private:
  // Receiver to monitor cuda stream activity
  Parameter<Handle<Receiver>> receiver_;
  // The current state of the scheduling term
  SchedulingConditionType current_state_;
  // Current message entity being monitored
  std::atomic<gxf_uid_t> message_eid_ = kNullUid;
  // State of the scheduling term
  std::atomic<State> state_{State::UNSET};
  // timestamp when the state changed the last time
  int64_t last_state_change_;
  // cuda host callback function on completion of all the queued work
  static void CUDART_CB cudaHostCallback(void* term_ptr);
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_CUDA_CUDA_SCHEDULING_TERMS_HPP_
