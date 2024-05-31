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
#ifndef NVIDIA_GXF_GXF_GRPC_GRPC_CLIENT_HPP_
#define NVIDIA_GXF_GXF_GRPC_GRPC_CLIENT_HPP_

#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "gxf/core/component.hpp"
#include "gxf/core/gxf.h"
#include "gxf/std/ipc_client.hpp"

namespace nvidia {
namespace gxf {

/**
 * An interprocess remote call client.
 * Grpc client implementation following GXF IPC Server-Client interface.
 * This IPC client is a complement to GrpcServer as IPC Server
*/
class GrpcClient : public IPCClient {
 public:
  gxf_result_t registerInterface(Registrar* registrar) override;
  gxf_result_t initialize() override;
  gxf_result_t deinitialize() override;

  GrpcClient& changeAddress(const std::string& ip, uint32_t port) override;
  Expected<std::string> query(const std::string& service,
                              const std::string& resource) override;
  Expected<void> action(const std::string& service,
                        const std::string& resource,
                        const std::string& data) override;
  Expected<std::string>
    ping(const std::string& service = kDefaultPingServiceName) override;

 private:
  Parameter<bool> enable_health_check_;
  class Impl;
  struct ImplDeleter {
    void operator()(Impl* ptr);
  };
  std::unique_ptr<Impl, ImplDeleter> impl_;
};

}  // namespace gxf
}  // namespace nvidia
#endif
