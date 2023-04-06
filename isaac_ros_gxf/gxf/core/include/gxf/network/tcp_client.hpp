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
#ifndef NVIDIA_GXF_NETWORK_TCP_CLIENT_HPP_
#define NVIDIA_GXF_NETWORK_TCP_CLIENT_HPP_

#include <string>
#include <unordered_map>
#include <vector>

#include "gxf/network/tcp_client_socket.hpp"
#include "gxf/serialization/entity_serializer.hpp"
#include "gxf/std/codelet.hpp"
#include "gxf/std/receiver.hpp"
#include "gxf/std/transmitter.hpp"

namespace nvidia {
namespace gxf {

// Codelet that functions as a client in a TCP connection
class TcpClient : public Codelet {
 public:
  gxf_result_t registerInterface(Registrar* registrar) override;
  gxf_result_t initialize() override;
  gxf_result_t deinitialize() override { return ToResultCode(client_socket_.close()); }

  gxf_result_t start() override;
  gxf_result_t tick() override;
  gxf_result_t stop() override { return GXF_SUCCESS; }

  const TcpClientSocket& getClientSocket() { return client_socket_; }

 private:
  Parameter<std::vector<Handle<Receiver>>> receivers_;
  Parameter<std::vector<Handle<Transmitter>>> transmitters_;
  Parameter<Handle<EntitySerializer>> entity_serializer_;
  Parameter<std::string> address_;
  Parameter<int> port_;
  Parameter<uint64_t> timeout_ms_;
  Parameter<uint64_t> maximum_attempts_;

  // Maps channel IDs to transmitters
  std::unordered_map<uint64_t, Handle<Transmitter>> channel_map_;
  // TCP client socket
  TcpClientSocket client_socket_;
  // Execution timestamp for measuring connection timeout
  int64_t timestamp_;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_NETWORK_TCP_CLIENT_HPP_
