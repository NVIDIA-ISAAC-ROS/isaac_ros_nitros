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
#include "extensions/utils/udp_receiver.hpp"

#include <sys/eventfd.h>
#include <sys/poll.h>

#include <utility>

#include "common/span.hpp"
#include "gxf/std/memory_buffer.hpp"
#include "gxf/std/tensor.hpp"

namespace nvidia {
namespace isaac {

namespace {

// Maximum size of a UDP packet
// Assumes a 2 byte size field (65535 bytes maximum)
// with 20 bytes reserved for IP header and 8 bytes reserved for UDP header
constexpr size_t kUdpMaxPacketSize = 65507;

}

gxf_result_t UdpReceiver::registerInterface(gxf::Registrar* registrar) {
  if (registrar == nullptr) {
    return GXF_ARGUMENT_NULL;
  }
  gxf::Expected<void> result;
  result &= registrar->parameter(
      tensor_, "tensor", "Tensor",
      "Tensor output");
  result &= registrar->parameter(
      allocator_, "allocator", "Allocator",
      "Memory allocator");
  result &= registrar->parameter(
      async_scheduling_term_, "async_scheduling_term", "Asynchronous Scheduling Term",
      "Schedules execution when socket has data available to read");
  result &= registrar->parameter(
      address_, "address", "Address",
      "IP address");
  result &= registrar->parameter(
      port_, "port", "Port",
      "Port number");
  result &= registrar->parameter(
      packet_accumulation_, "packet_accumulation", "Packet Accumulation",
      "Number of packets to accumulate before publishing",
      1UL);
  result &= registrar->parameter(
      buffer_size_, "buffer_size", "Buffer Size",
      "Read buffer size in bytes",
      kUdpMaxPacketSize);
  result &= registrar->parameter(
      receive_buffer_size_, "receive_buffer_size", "Receive Buffer Size",
      "UDP receive buffer size in bytes (overrides value in /proc/sys/net/core/rmem_default)",
      gxf::Registrar::NoDefaultParameter(), GXF_PARAMETER_FLAGS_OPTIONAL);
  return gxf::ToResultCode(result);
}

gxf_result_t UdpReceiver::initialize() {
  if (packet_accumulation_ == 0) {
    GXF_LOG_ERROR("packet_accumulation must be greater than 0");
    return GXF_PARAMETER_OUT_OF_RANGE;
  }
  if (buffer_size_ == 0 || buffer_size_ > kUdpMaxPacketSize) {
    GXF_LOG_ERROR("buffer_size must be greater than 0 and less than or equal to %zu",
                  kUdpMaxPacketSize);
    return GXF_PARAMETER_OUT_OF_RANGE;
  }
  return GXF_SUCCESS;
}

gxf_result_t UdpReceiver::start() {
  // Create UDP socket
  socket_.reset(::nvidia::isaac::Socket::CreateRxUDPSocket(address_, port_));
  int result = socket_->startSocket();
  if (result < 0) {
    GXF_LOG_ERROR("Failed to start socket");
    return GXF_FAILURE;
  }

  auto receive_buffer_size = receive_buffer_size_.try_get();
  if (receive_buffer_size) {
    int opt = static_cast<int>(receive_buffer_size.value());
    uint32_t length = sizeof(opt);
    result = setsockopt(socket_->getFileDescriptor(), SOL_SOCKET, SO_RCVBUF, &opt, length);
    if (result != 0) {
      GXF_LOG_ERROR("%s", strerror(errno));
      return GXF_FAILURE;
    }
    result = getsockopt(socket_->getFileDescriptor(), SOL_SOCKET, SO_RCVBUF, &opt, &length);
    if (result != 0) {
      GXF_LOG_ERROR("%s", strerror(errno));
      return GXF_FAILURE;
    }
    // Configured size is doubled by kernel for bookkeeping so divide actual size by 2
    opt /= 2;
    if (static_cast<size_t>(opt) != receive_buffer_size.value()) {
      // If the configured buffer size does not match the requested buffer size,
      // then the requested amount is likely greater than the value specified in
      // /proc/sys/net/core/rmem_max
      GXF_LOG_WARNING("UDP receive buffer is %dB but user requested %luB",
                      opt, receive_buffer_size.value());
    }
  }

  // Create event file descriptor
  event_fd_ = eventfd(0, 0);
  if (event_fd_ < 0) {
    GXF_LOG_ERROR("%s", strerror(errno));
    return GXF_FAILURE;
  }

  // Start async thread
  thread_ = std::thread([this] { asyncSocketMonitor(); });

  return GXF_SUCCESS;
}

gxf_result_t UdpReceiver::tick() {
  // Create message entity
  auto entity = gxf::Entity::New(context());
  if (!entity) {
    return gxf::ToResultCode(entity);
  }

  // Accumulate packets and add them to message entity as tensors
  for (size_t packet = 0; packet < packet_accumulation_; packet++) {
    gxf::MemoryBuffer buffer;
    gxf::Handle<gxf::Tensor> tensor;
    auto result = entity->add<gxf::Tensor>()
        .assign_to(tensor)
        .and_then([&]() {
          return buffer.resize(allocator_, buffer_size_, gxf::MemoryStorageType::kSystem);
        })
        .and_then([&]() -> gxf::Expected<int> {
          Span<char> span(reinterpret_cast<char*>(buffer.pointer()), buffer.size());
          // This call will block if there is no data available on the socket
          const int received = socket_->readPacket(span.data(), span.size());
          if (received < 0) {
            GXF_LOG_ERROR("Failed to read from socket");
            return gxf::Unexpected{GXF_FAILURE};
          }
          return received;
        })
        .map([&](int size) {
          return tensor->wrapMemoryBuffer({size},
                                          gxf::PrimitiveType::kUnsigned8,
                                          PrimitiveTypeSize(gxf::PrimitiveType::kUnsigned8),
                                          gxf::Unexpected{GXF_UNINITIALIZED_VALUE},
                                          std::move(buffer));
        });
    if (!result) {
      return gxf::ToResultCode(result);
    }
  }

  // Put main thread in a waiting state until socket has data to read
  async_scheduling_term_->setEventState(gxf::AsynchronousEventState::EVENT_WAITING);
  const uint64_t state = static_cast<uint64_t>(async_scheduling_term_->getEventState());
  const ssize_t bytes = write(event_fd_, &state, sizeof(state));
  if (bytes < 0) {
    GXF_LOG_ERROR("Failed to write file descriptor: %s", strerror(errno));
    return GXF_FAILURE;
  }

  // Publish message entity with timestamp
  return gxf::ToResultCode(tensor_->publish(entity.value(), getExecutionTimestamp()));
}

gxf_result_t UdpReceiver::stop() {
  // Stop async thread and wait for exit
  async_scheduling_term_->setEventState(gxf::AsynchronousEventState::EVENT_NEVER);
  const uint64_t state = static_cast<uint64_t>(async_scheduling_term_->getEventState());
  const ssize_t bytes = write(event_fd_, &state, sizeof(state));
  if (bytes < 0) {
    GXF_LOG_ERROR("Failed to write file descriptor: %s", strerror(errno));
    return GXF_FAILURE;
  }
  thread_.join();

  // Close event file descriptor
  const int result = close(event_fd_);
  if (result != 0) {
    GXF_LOG_ERROR("%s", strerror(errno));
    return GXF_FAILURE;
  }

  // Close UDP socket
  socket_->closeSocket();

  return GXF_SUCCESS;
}

void UdpReceiver::asyncSocketMonitor() {
  // Poll structure for state
  pollfd state_fds;
  state_fds.fd = event_fd_;
  state_fds.events = POLLIN;

  // Poll structure for socket
  pollfd socket_fds;
  socket_fds.fd = socket_->getFileDescriptor();
  socket_fds.events = POLLIN;

  while (true) {
    // Block until main thread signals a state change
    int result = poll(&state_fds, 1, -1);
    if (result < 0) {
      LOG_ERROR("%s", strerror(errno));
    }

    if (state_fds.revents & POLLIN) {
      // Read current state from main thread
      uint64_t state;
      const ssize_t bytes = read(state_fds.fd, &state, sizeof(state));
      if (bytes < 0) {
        GXF_LOG_ERROR("%s", strerror(errno));
      }

      // Stop async thread if codelet is stopped
      if (state == static_cast<uint64_t>(gxf::AsynchronousEventState::EVENT_NEVER)) {
        break;
      }

      // Main thread should be done reading from socket before polling
      if (state != static_cast<uint64_t>(gxf::AsynchronousEventState::EVENT_WAITING)) {
        continue;
      }
    }

    // Block until data is available to read on the socket
    result = poll(&socket_fds, 1, -1);
    if (result < 0) {
      LOG_ERROR("%s", strerror(errno));
    }

    // Change scheduling state if there is data available to read
    if (socket_fds.revents & POLLIN) {
      async_scheduling_term_->setEventState(gxf::AsynchronousEventState::EVENT_DONE);
    }
  }
}

}  // namespace isaac
}  // namespace nvidia
