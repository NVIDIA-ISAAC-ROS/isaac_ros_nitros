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

#ifndef NVIDIA_GXF_APPLICATION_DRIVER_HPP_
#define NVIDIA_GXF_APPLICATION_DRIVER_HPP_

#include <memory>
#include <string>

#include "gxf/app/segment.hpp"
#include "gxf/std/graph_driver.hpp"

namespace nvidia {
namespace gxf {

class Application;

/**
 * @brief GraphDriver representation in Application API layer
 * It drives the execution of all remote GraphWorkers
 *
 */
class Driver {
 public:
  Driver(Application* owner, const std::string& name);

  std::string name() { return name_; }
  Expected<void> setPort(uint32_t port) { port_ = port; return Success; }
  // commit the setup into GraphWorker
  Expected<void> commit();
 private:
  Application* owner_;
  std::string name_;
  GraphEntityPtr driver_entity_;
  // core component
  Handle<GraphDriver> graph_driver_;
  // server interface object
  Handle<IPCServer> server_;
  // client interface object
  Handle<IPCClient> client_;

  // driver server's own port
  uint32_t port_;
};
typedef std::shared_ptr<Driver> DriverPtr;

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_APPLICATION_DRIVER_HPP_
