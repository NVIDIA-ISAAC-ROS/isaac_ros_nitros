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
#ifndef NVIDIA_GXF_GXF_HTTP_HTTP_IPC_CLIENT_HPP_
#define NVIDIA_GXF_GXF_HTTP_HTTP_IPC_CLIENT_HPP_

#include <memory>
#include <string>

#include "gxf/core/gxf.h"
#include "gxf/std/ipc_client.hpp"

namespace nvidia {
namespace gxf {

/**
 * A gxf::IPCClient that is implemented in HTTP
 * This client works in pair with gxf::HttpServer
*/
class HttpIPCClient : public IPCClient {
 public:
  gxf_result_t registerInterface(Registrar* registrar) override;
  gxf_result_t initialize() override;
  gxf_result_t deinitialize() override;

  HttpIPCClient& changeAddress(const std::string& ip, uint32_t port) override;
  Expected<std::string> query(const std::string& service,
                              const std::string& resource) override;
  Expected<void> action(const std::string& service,
                        const std::string& resource,
                        const std::string& data) override;
  Expected<std::string>
    ping(const std::string& service = kDefaultPingServiceName) override;

 private:
  Parameter<bool> use_https_;
  Parameter<std::string> content_type_;
  class Impl;
  struct ImplDeleter {
    void operator()(Impl* ptr);
  };
  std::unique_ptr<Impl, ImplDeleter> client_;

  typedef struct {
    int status_code;
    std::string body;
  } Response;
};

}  // namespace gxf
}  // namespace nvidia
#endif
