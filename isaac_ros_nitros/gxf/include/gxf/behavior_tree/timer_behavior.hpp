/*
 * SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef NVIDIA_GXF_BT_TIMER_BEHAVIOR_HPP_
#define NVIDIA_GXF_BT_TIMER_BEHAVIOR_HPP_

#include "gxf/std/clock.hpp"
#include "gxf/std/codelet.hpp"
#include "gxf/std/controller.hpp"
#include "gxf/std/scheduling_terms.hpp"

namespace nvidia {
namespace gxf {

// Timer Behavior
// Waits for a specified amount of time delay,
// and switches to the configured result switch_status afterwards.
class TimerBehavior : public Codelet {
 public:
  virtual ~TimerBehavior() = default;

  gxf_result_t registerInterface(Registrar* registrar) override;
  gxf_result_t initialize() override;
  gxf_result_t start() override { return GXF_SUCCESS; }
  gxf_result_t tick() override;
  gxf_result_t stop() override { return GXF_SUCCESS; }

 private:
  // a specified time (in seconds) after which the status of this node will be
  // changed
  Parameter<double> delay_;

  // the desired status to switch to when the tick time is delay_ seconds after
  // first tick Setting this value to “running” will lead to undefined behavior
  // in the application.
  Parameter<size_t> switch_status_;
  enum SwitchStatus {
    kSwitchToSuccess = 0,
    kSwitchToFailure = 1,
    kSwitchToRunning = 2,
  };

  // Use this clock to get wait time since first tick
  Parameter<Handle<Clock>> clock_;

  // its own scheduling term to start/stop itself
  Parameter<Handle<nvidia::gxf::BTSchedulingTerm>> s_term_;

  bool is_first_tick_ = true;

  // timestamp of the last tick which caused a status switch
  int64_t last_switch_timestamp_ = 0;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_BT_TIMER_BEHAVIOR_HPP_
