// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#ifndef NVIDIA_GXF_GXF_GRPC_GRPC_SERVER_HPP_
#define NVIDIA_GXF_GXF_GRPC_GRPC_SERVER_HPP_

#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "gxf/core/component.hpp"
#include "gxf/core/gxf.h"
#include "gxf/std/ipc_server.hpp"

namespace nvidia {
namespace gxf {

class ServiceImpl;

class GrpcServer : public IPCServer {
 public:
  static constexpr const char* kGrpcMainEntranceService = "ServiceHub";
  gxf_result_t registerInterface(Registrar* registrar) override;

  gxf_result_t initialize() override;

  gxf_result_t deinitialize() override;

  Expected<void> registerService(const IPCServer::Service& service) override;

 private:
  Parameter<uint32_t> port_;
  Parameter<bool> remote_access_;
  Parameter<bool> enable_health_check_;
  std::map<std::string, IPCServer::QueryHandler> query_handlers_;
  std::map<std::string, IPCServer::ActionHandler> action_handlers_;
  std::thread thread_;
  std::mutex mutex_;
  class Impl;
  struct ImplDeleter {
    void operator()(Impl* ptr);
  };
  std::unique_ptr<Impl, ImplDeleter> impl_;

  template<typename T>
  Expected<void> installHandler(std::map<std::string, T>& dest,
                                const std::string& path, T handler) {
    Expected<void> result;
    std::unique_lock<std::mutex> lock(mutex_);
    if (dest.find(path) != dest.end()) {
      return Unexpected{GXF_FAILURE};
    }
    dest[path] = handler;
    return result;
  }

  Expected<std::string> onPing(const std::string& resource) {
    return resource + " is good\n";
  }

  Expected<void> handleRequest(const std::string & service,
                             std::vector<std::string> params,
                             std::string& result);

  Expected<void> setServingStatus(const std::string& service_name, bool serving);
  Expected<void> setAllRegisteredServiceStatus(bool serving);

friend ServiceImpl;
};

}  // namespace gxf
}  // namespace nvidia
#endif
