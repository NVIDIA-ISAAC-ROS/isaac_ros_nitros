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
#ifndef NVIDIA_GXF_NETWORK_TCP_CLIENT_HPP_
#define NVIDIA_GXF_NETWORK_TCP_CLIENT_HPP_

#include "gxf/network/tcp_codelet.hpp"

namespace nvidia {
namespace gxf {

// Codelet that functions as a client in a TCP connection
class TcpClient : public TcpCodelet {
 protected:
  Expected<void> openSockets() override;
  Expected<void> reconnectSockets() override;
  Expected<void> closeSockets() override;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_NETWORK_TCP_CLIENT_HPP_
