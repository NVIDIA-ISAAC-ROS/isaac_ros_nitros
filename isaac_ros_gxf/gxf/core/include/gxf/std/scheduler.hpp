// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#pragma once

#include <unistd.h>
#include <memory>
#include <vector>
#include "gxf/core/gxf.h"
#include "gxf/std/resources.hpp"
#include "gxf/std/system.hpp"

namespace nvidia {
namespace gxf {

class EntityExecutor;  // forward declaration

// An interface which extends the nvidia::gxf::System interface to create schedulers
// which can execute codelets.

class Scheduler : public System {
 public:
  virtual gxf_result_t prepare_abi(EntityExecutor* executor) = 0;
};

#ifdef __linux__

// glibc before 2.41 does not provide wrappers for sched_setattr() and sched_getattr(),
// so we need to define them here.
#ifdef __x86_64__
#define __NR_sched_setattr 314
#define __NR_sched_getattr 315
#endif

#ifdef __aarch64__
#define __NR_sched_setattr 274
#define __NR_sched_getattr 275
#endif

struct sched_attr {
  uint32_t size;
  uint32_t sched_policy;
  uint64_t sched_flags;
  int32_t sched_nice;
  uint32_t sched_priority;
  uint64_t sched_runtime;
  uint64_t sched_deadline;
  uint64_t sched_period;
};

static inline int
sched_setattr(pid_t pid, const struct sched_attr* attr, unsigned flags) {
    return syscall(__NR_sched_setattr, pid, attr, flags);
}

static inline int
sched_getattr(pid_t pid, struct sched_attr* attr,
              unsigned int size, unsigned int flags) {
    return syscall(__NR_sched_getattr, pid, attr, size, flags);
}
#endif

}  // namespace gxf
}  // namespace nvidia
