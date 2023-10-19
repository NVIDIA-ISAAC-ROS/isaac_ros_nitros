// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#ifndef NVIDIA_GXF_STD_RESOURCES_HPP_
#define NVIDIA_GXF_STD_RESOURCES_HPP_

#include <map>
#include <string>

#include "gxf/core/component.hpp"
#include "gxf/core/entity.hpp"

namespace nvidia {
namespace gxf {

// A base type for all kinds of resources.
// This is used to filter out resource Component from an entity
class ResourceBase : public Component {};

class ThreadPool : public ResourceBase {
 public:
  struct Thread {
    gxf_uid_t uid;
    // Scalability: std::thread
  };
  gxf_result_t registerInterface(Registrar* registrar) override;
  gxf_result_t initialize() override;
  Expected<gxf_uid_t> addThread(gxf_uid_t uid);
  const Expected<Thread> getThread(gxf_uid_t uid) const;
  const std::map<gxf_uid_t, Thread>& get() const;
  int64_t size() const;
  int64_t priority() const;

 private:
  Parameter<int64_t> initial_size_;
  Parameter<int64_t> priority_;
  std::map<gxf_uid_t, Thread> thread_pool_;
};

class GPUDevice : public ResourceBase {
 public:
  gxf_result_t registerInterface(Registrar* registrar) override;
  int32_t device_id() const { return dev_id_; }

 private:
  Parameter<int32_t> dev_id_;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_STD_RESOURCES_HPP_
