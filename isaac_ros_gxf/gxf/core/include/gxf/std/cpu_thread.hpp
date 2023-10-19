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
#ifndef NVIDIA_GXF_STD_CPU_THREAD_HPP
#define NVIDIA_GXF_STD_CPU_THREAD_HPP

#include "gxf/core/component.hpp"
#include "gxf/std/resources.hpp"

namespace nvidia {
namespace gxf {

class CPUThread : public Component {
 public:
  gxf_result_t registerInterface(Registrar* registrar) override;

  bool pinned() const {
    return pin_entity_;
  }

 private:
  // Keep track of whether or not the component should be pinned to a worker thread
  Parameter<bool> pin_entity_;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_STD_CPU_THREAD_HPP
