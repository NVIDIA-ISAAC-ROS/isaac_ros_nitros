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

#ifndef NVIDIA_GXF_GXF_STD_HTTP_SERVER_HPP_
#define NVIDIA_GXF_GXF_STD_HTTP_SERVER_HPP_

#include <map>
#include <memory>
#include <mutex>
#include <string>

#include "gxf/core/component.hpp"
#include "gxf/core/gxf.h"
#include "gxf/std/ipc_server.hpp"

#include "cpprest/http_listener.h"

namespace nvidia {
namespace gxf {

class HttpServer : public IPCServer {
 public:
  gxf_result_t registerInterface(Registrar* registrar) override;

  gxf_result_t initialize() override;

  gxf_result_t deinitialize() override;

  Expected<void> registerService(const IPCServer::Service& service) {
    if (service.type == IPCServer::kQuery) {
      return installHandler(get_handlers_, service.name, service.handler.query);
    } else if (service.type == IPCServer::kAction) {
      return installHandler(post_handlers_, service.name, service.handler.action);
    } else {
      return Unexpected{GXF_FAILURE};
    }
  }

 private:
  std::unique_ptr<web::http::experimental::listener::http_listener> listener_;
  std::map<std::string, IPCServer::QueryHandler> get_handlers_;
  std::map<std::string, IPCServer::ActionHandler> post_handlers_;
  std::mutex mutex_;

  Parameter<uint32_t> port_;
  Parameter<bool> remote_access_;

  Expected<std::string> onPing(const std::string& resource) {
    return resource + " is good\n";
  }

  template<typename T>
  Expected<void> installHandler(
    std::map<std::string, T>& dest,
    const std::string& path,
    T handler
  ) {
    Expected<void> result;
    std::unique_lock<std::mutex> lock(mutex_);
    if (dest.find(path) != dest.end()) {
        return Unexpected{GXF_FAILURE};
    }
    dest[path] = handler;
    return result;
  }

  void handleRequest(web::http::http_request);
  using status_codes = web::http::status_codes;
  using uri = web::http::uri;
  using methods = web::http::methods;
  using listener = web::http::experimental::listener::http_listener;
};

}  // namespace gxf
}  // namespace nvidia
#endif
