// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2021-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef NVIDIA_GXF_NETWORK_TCP_SERVER_SOCKET_HPP_
#define NVIDIA_GXF_NETWORK_TCP_SERVER_SOCKET_HPP_

#include <string>

#include "gxf/network/tcp_client_socket.hpp"

namespace nvidia {
namespace gxf {

// Object for managing a TCP socket on the server side.
// Connection must be established prior to use.
// Uses a TcpClientSocket as a proxy for communication.
class TcpServerSocket {
 public:
  TcpServerSocket(const char* address, uint16_t port)
    : address_{address}, port_{port}, socket_{-1} {}
  TcpServerSocket() : address_{"0.0.0.0"}, port_{0}, socket_{-1} {}
  ~TcpServerSocket() = default;
  TcpServerSocket(const TcpServerSocket& other) = delete;
  TcpServerSocket(TcpServerSocket&& other) = default;
  TcpServerSocket& operator=(const TcpServerSocket& other) = delete;
  TcpServerSocket& operator=(TcpServerSocket&& other) = default;

  // Initializes server socket.
  // Creates socket file descriptor.
  // Binds IP address to socket.
  // Listens for incoming connections on socket.
  Expected<void> open();
  // Deinitializes server socket.
  // Destroys socket file descriptor.
  Expected<void> close();
  // Attempts to connect to a TCP client.
  // Returns a TCP client socket that can be used as an endpoint.
  // Call will fail if there are no clients requesting to connect.
  Expected<TcpClientSocket> connect();

 private:
  // Server address
  std::string address_;
  // Server port
  uint16_t port_;
  // Socket file descriptor
  int socket_;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_NETWORK_TCP_SERVER_SOCKET_HPP_
