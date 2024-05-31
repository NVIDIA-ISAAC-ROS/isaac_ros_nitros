// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2018-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#pragma once

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <linux/if.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <functional>
#include <string>

#include "gems/core/logger.hpp"

namespace nvidia {
namespace isaac {

class Socket {
 public:
  // Creates a UDP receiving socket
  // Remote address corresponds to the sending socket. Port will be used for remote and local port
  static Socket* CreateRxUDPSocket(const std::string& remote_address, uint16_t port) {
    Socket* s = new Socket(remote_address, port, SOCK_DGRAM);
    memset(&s->local_address_, 0, sizeof(s->local_address_));
    s->local_address_.sin_port   = htons(port);
    s->local_address_.sin_family = AF_INET;
    s->local_address_.sin_addr.s_addr = htonl(INADDR_ANY);
    return s;
  }

  // Creates a TCP receiving socket
  static Socket* CreateRxTCPSocket(const std::string& source_address, uint16_t source_port) {
    return new Socket(source_address, source_port, SOCK_STREAM);
  }

  // Try to start a socket or fails and return -1
  int32_t startSocket() {
    int32_t res = 0;

    // TODO Assert
    if (type_ == SOCK_STREAM) {
      res = connect(sockfd_, (struct sockaddr *)&emitter_address_,
              sizeof(emitter_address_));
      LOG_WARNING("Never tested");
      // Set reading options
      read_flags_ = MSG_PEEK | MSG_WAITALL;
    } else if (type_ == SOCK_DGRAM) {
      uint32_t reuse = 1;
      // Set Reuse Option
      struct timeval tv;
      tv.tv_sec = 1;
      tv.tv_usec = 0;
      res |= setsockopt(sockfd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
      res |= setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(uint32_t));
      if (res < 0) {
        LOG_ERROR("Cannot set Socket options");
      }
      if (bind(sockfd_, (struct sockaddr *)&local_address_, sizeof(local_address_)) == -1) {
        LOG_ERROR("Cannot bind!");
        return -1;
      }
      read_flags_ = 0;
    }
    is_running_ = true;
    return res;
  }

  // Reads a packet
  size_t readPacket(char* data, size_t size) {
    if (!sockfd_) {
      LOG_ERROR("Socket is invalid, reopen the connection!");
      return -1;
    }
    socklen_t slen = sizeof(emitter_address_);
    int32_t recvlen = recvfrom(sockfd_, data, size, 0,
                  (struct sockaddr *)&emitter_address_, &slen);
    return recvlen;
  }

  // Reads a packet from
  size_t writePacket(const void* data, size_t size) {
    if (!sockfd_) {
      LOG_ERROR("Socket is invalid, reopen the connection!");
      return -1;
    }
    socklen_t slen = sizeof(emitter_address_);
    int32_t recvlen = sendto(sockfd_, data, size, 0,
                            (struct sockaddr *)&emitter_address_, slen);
    return recvlen;
  }

  // Closes socket
  void closeSocket() {
    if (sockfd_) {
      close(sockfd_);
    }
    is_running_ = false;
  }

  bool isRunning() {
    return is_running_;
  }

  // Identify the (externally exposed) IPv4 address corresponding to this communication socket.
  uint32_t getIpv4() const {
    uint32_t local_ipv4_address(0);

    // For each connected IPv4 network interface address, if the emitter address meets masking
    // requirement, we found our local IPv4. Don't bother checking if the socket is not running.
    struct ifaddrs* network_interface_addresses = nullptr;
    if (getifaddrs(&network_interface_addresses) != -1) {
      struct ifaddrs* network_interface_addresses_iterator = network_interface_addresses;
      while (network_interface_addresses_iterator != nullptr) {
        // Validate the socket family is indeed IPv4.
        if ((network_interface_addresses_iterator->ifa_addr != nullptr) &&
            (network_interface_addresses_iterator->ifa_addr->sa_family == AF_INET)) {
          // Validate the socket is actually connected.
          struct sockaddr_in* local_socket_address =
              (struct sockaddr_in*)network_interface_addresses_iterator->ifa_addr;
          if (local_socket_address->sin_addr.s_addr != htonl(INADDR_ANY)) {
            // Validate the network masking requirements meets our emitter address.
            struct sockaddr_in* ifu_netmask =
                (struct sockaddr_in*)network_interface_addresses_iterator->ifa_netmask;
            if ((local_socket_address->sin_addr.s_addr & ifu_netmask->sin_addr.s_addr) ==
                (emitter_address_.sin_addr.s_addr & ifu_netmask->sin_addr.s_addr)) {
              local_ipv4_address = local_socket_address->sin_addr.s_addr;
              break;
            }
          }
        }
        network_interface_addresses_iterator = network_interface_addresses_iterator->ifa_next;
      }

      if (network_interface_addresses) {
        freeifaddrs(network_interface_addresses);
      }
    }

    return local_ipv4_address;
  }

  int32_t getFileDescriptor() const {
    return sockfd_;
  }

 private:
  int32_t sockfd_;
  struct sockaddr_in local_address_;
  struct sockaddr_in emitter_address_;

  int32_t read_flags_;
  int32_t type_;
  bool is_running_;

  Socket(const std::string& remote_address, uint16_t remote_port, uint16_t type,
       const std::string& interface = "") {
    // Prepares for incoming data
    memset(&emitter_address_, 0, sizeof(emitter_address_));
    emitter_address_.sin_port   = htons(remote_port);
    emitter_address_.sin_family = AF_INET;
    emitter_address_.sin_addr.s_addr = inet_addr(remote_address.c_str());
    type_ = type;
    is_running_ = false;

    // Creates the socket
    sockfd_ = socket(AF_INET, type, 0);

    if (!interface.empty()) {
      struct ifreq if_bind;
      strncpy(if_bind.ifr_name, interface.c_str(), IFNAMSIZ);
      const int error = setsockopt(sockfd_, SOL_SOCKET, SO_BINDTODEVICE,
                                   reinterpret_cast<char*>(&if_bind), sizeof(if_bind));
      if (error < 0) {
        LOG_ERROR("Failed to specify interface %s", interface.c_str());
      }
    }
  }
};

}  // namespace isaac
}  // namespace nvidia
