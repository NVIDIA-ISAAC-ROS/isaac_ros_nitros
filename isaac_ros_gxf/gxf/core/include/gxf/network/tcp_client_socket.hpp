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

#ifndef NVIDIA_GXF_NETWORK_TCP_CLIENT_SOCKET_HPP_
#define NVIDIA_GXF_NETWORK_TCP_CLIENT_SOCKET_HPP_

#include <sys/poll.h>

#include <utility>
#include <vector>

#include "gxf/serialization/endpoint.hpp"
#include "gxf/serialization/entity_serializer.hpp"

namespace nvidia {
namespace gxf {

// Header containing metadata for TCP packets
#pragma pack(push, 1)
struct TcpHeader {
  uint64_t payload_size;  // Size of the payload in bytes
  uint64_t entity_count;  // Number of entities in the payload
};
#pragma pack(pop)

// Header containing metadata for entities
#pragma pack(push, 1)
struct MessageHeader {
  uint64_t channel_id;  // Hashed channel name
};
#pragma pack(pop)

// TCP message consisting of a header and an entity
struct TcpMessage {
  MessageHeader header;
  Entity entity;
};

// Endpoint for exchanging data over a TCP connection.
// Connection must be established prior to use.
// Packets are organized in the following format:
//
//   | TCP Header || Message Header | Entity | ... | ... |
//
// Each entity will have a header prepended to form a message.
// Messages will be grouped together to form the packet.
// Little-endian is used over big-endian for better performance on x86 and arm platforms.
class TcpClientSocket : public Endpoint {
 public:
  TcpClientSocket() = default;
  ~TcpClientSocket() = default;
  TcpClientSocket(const TcpClientSocket& other) = delete;
  TcpClientSocket(TcpClientSocket&& other) { *this = std::move(other); }
  TcpClientSocket& operator=(const TcpClientSocket& other) = delete;
  TcpClientSocket& operator=(TcpClientSocket&& other) {
    fd_socket_ = other.fd_socket_;
    connected_ = other.connected_;
    maximum_attempts_ = other.maximum_attempts_;
    other.fd_socket_ = -1;
    other.connected_ = false;
    other.maximum_attempts_ = 1;
    return *this;
  }

  Expected<void> open();
  Expected<void> close();

  gxf_result_t write_abi(const void* data, size_t size, size_t* bytes_written) override;
  gxf_result_t read_abi(void* data, size_t size, size_t* bytes_read) override;

  // Initializes endpoint with a connected socket.
  // This is used by TcpServerSocket when establishing a connection.
  Expected<void> openConnectedSocket(int socket);
  // Closes and reopens socket to enable reconnecting.
  // Socket needs to be reset after disconnecting to be used again.
  Expected<void> reopenSocket();
  // Configures the maximum number of attempts
  void setMaximumAttempts(size_t attempts) { maximum_attempts_ = attempts; }
  // Returns true if the socket has data available for reading
  bool available() const;
  // Returns true if the socket is connected
  bool connected() const { return connected_; }
  // Attempts to connect to a TCP server at the given IP address.
  // Call will fail if IP address is invalid or server is unreachable.
  Expected<void> connect(const char* address, uint16_t port);

  // Sends a set of messages via TCP
  Expected<size_t> sendMessages(const std::vector<TcpMessage>& messages,
                                EntitySerializer* serializer);
  // Receives a set of messages via TCP
  Expected<std::vector<TcpMessage>> receiveMessages(gxf_context_t context,
                                                    EntitySerializer* serializer);

 private:
  // Sends a message to the socket
  Expected<size_t> sendMessage(const TcpMessage& message, EntitySerializer* serializer);
  // Receives a message from the socket
  Expected<TcpMessage> receiveMessage(gxf_context_t context, EntitySerializer* serializer);
  // Sends a TCP header to the socket
  Expected<size_t> sendTcpHeader(TcpHeader header);
  // Receives a TCP header from the socket
  Expected<TcpHeader> receiveTcpHeader();
  // Sends a message header to the socket
  Expected<size_t> sendMessageHeader(MessageHeader header);
  // Receives a message header from the socket
  Expected<MessageHeader> receiveMessageHeader();

  // Socket file descriptor
  int fd_socket_{-1};

  // TCP connection state
  bool connected_{false};
  // Maximum number of attempts for each I/O operation
  size_t maximum_attempts_{1};
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_NETWORK_TCP_CLIENT_SOCKET_HPP_
