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
#ifndef NVIDIA_GXF_GXF_STD_IPC_CLIENT_HPP_
#define NVIDIA_GXF_GXF_STD_IPC_CLIENT_HPP_

#include <string>

#include "gxf/core/component.hpp"
#include "gxf/core/gxf.h"

namespace nvidia {
namespace gxf {

/**
 * interface of client to IPCServer
*/
class IPCClient : public Component {
 public:
  static constexpr const char* kDefaultPingServiceName = "ping";
  /**
   * param:
   *   rervice: object that implements the query API
   *   resource: on which the query is to be performed
   * return: text response that carries the result of a query
  */
  virtual Expected<std::string> query(const std::string& service,
                                      const std::string& resource) = 0;
  /**
   * param:
   *   service: object that implements the action API
   *   resource: on which the action is to be performed
   *   data: action data
   * return: indicator of a success or failure
  */
  virtual Expected<void> action(const std::string& service,
                                const std::string& resource,
                                const std::string& data) = 0;
  virtual Expected<std::string>
    ping(const std::string& service = kDefaultPingServiceName) = 0;
  virtual IPCClient& changeAddress(const std::string& ip, uint32_t port) = 0;

 protected:
  Parameter<uint32_t> port_;
  Parameter<std::string> server_ip_address_;
  std::string toIpPort(const std::string& ip, uint32_t port) {
    return ip + ":" + std::to_string(port);
  }
};

}  // namespace gxf
}  // namespace nvidia

#endif
