// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2021-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#ifndef NVIDIA_GXF_NETWORK_TCP_CODELET_HPP_
#define NVIDIA_GXF_NETWORK_TCP_CODELET_HPP_

#include <future>
#include <string>
#include <unordered_map>
#include <vector>

#include "gxf/network/tcp_server_socket.hpp"
#include "gxf/serialization/entity_serializer.hpp"
#include "gxf/std/codelet.hpp"
#include "gxf/std/receiver.hpp"
#include "gxf/std/scheduling_terms.hpp"
#include "gxf/std/transmitter.hpp"

namespace nvidia {
namespace gxf {

// Abstract codelet supporting asynchronous TCP client/server
class TcpCodelet : public Codelet {
 public:
  gxf_result_t registerInterface(Registrar* registrar) override;
  gxf_result_t initialize() override;
  gxf_result_t deinitialize() override;

  gxf_result_t start() override { return GXF_SUCCESS; }
  gxf_result_t tick() override;
  gxf_result_t stop() override;

 protected:
  // Asynchronously monitors client socket and receivers. If data is available from either source,
  // schedules the TcpServer via the AsynchronousSchedulingTerm. Also handles connecting client
  // socket.
  Expected<void> monitor();

  virtual Expected<void> openSockets() = 0;
  virtual Expected<void> reconnectSockets() = 0;
  virtual Expected<void> closeSockets() = 0;

  Parameter<std::vector<Handle<Receiver>>> receivers_;
  Parameter<std::vector<Handle<Transmitter>>> transmitters_;
  Parameter<Handle<EntitySerializer>> entity_serializer_;
  Parameter<std::string> address_;
  Parameter<int> port_;
  Parameter<uint64_t> timeout_ms_;
  Parameter<std::string> timeout_text_;
  Parameter<uint64_t> maximum_attempts_;
  Parameter<int64_t> max_msg_delay_ms_;
  Parameter<int64_t> max_duration_ms_;
  Parameter<uint64_t> max_connection_attempts_;
  Parameter<Handle<AsynchronousSchedulingTerm>> async_scheduling_term_;

  int64_t timeout_ns_;

  // Maps channel IDs to transmitters
  std::unordered_map<uint64_t, Handle<Transmitter>> channel_map_;
  // TCP client socket
  TcpClientSocket client_socket_;
  // Tracks monitor thread result
  std::future<Expected<void>> monitor_future_;
  // Stores receivers with available data
  std::vector<Handle<Receiver>> available_receivers_;
  // Stores messages read from receivers
  std::vector<TcpMessage> rx_messages_;
  // Stores messages read from socket
  std::vector<TcpMessage> tx_messages_;
  // Time when last message was received
  Expected<std::chrono::time_point<std::chrono::steady_clock>> last_msg_timestamp_ =
      Unexpected{GXF_UNINITIALIZED_VALUE};
  // Time when monitor thread began
  Expected<std::chrono::time_point<std::chrono::steady_clock>> monitor_start_timestamp_ =
      Unexpected{GXF_UNINITIALIZED_VALUE};
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_NETWORK_TCP_CODELET_HPP_
