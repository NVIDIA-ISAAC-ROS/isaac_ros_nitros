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
#pragma once

#include <memory>
#include <string>
#include <thread>

#include "gems/coms/socket.hpp"
#include "gxf/std/allocator.hpp"
#include "gxf/std/codelet.hpp"
#include "gxf/std/scheduling_terms.hpp"
#include "gxf/std/transmitter.hpp"

namespace nvidia {
namespace isaac {

// Network interface codelet that receives data from a UDP socket and publishes the data as a tensor
class UdpReceiver : public gxf::Codelet {
 public:
  gxf_result_t registerInterface(gxf::Registrar* registrar) override;
  gxf_result_t initialize() override;
  gxf_result_t deinitialize() override { return GXF_SUCCESS; }

  gxf_result_t start() override;
  gxf_result_t tick() override;
  gxf_result_t stop() override;

 private:
  // Asynchronous thread that monitors when the socket has data available to read
  void asyncSocketMonitor();

  gxf::Parameter<gxf::Handle<gxf::Transmitter>> tensor_;
  gxf::Parameter<gxf::Handle<gxf::Allocator>> allocator_;
  gxf::Parameter<gxf::Handle<gxf::AsynchronousSchedulingTerm>> async_scheduling_term_;
  gxf::Parameter<std::string> address_;
  gxf::Parameter<uint16_t> port_;
  gxf::Parameter<size_t> packet_accumulation_;
  gxf::Parameter<size_t> buffer_size_;
  gxf::Parameter<size_t> receive_buffer_size_;

  // UDP socket to receive data from
  std::unique_ptr<::nvidia::isaac::Socket> socket_;
  // Async thread
  std::thread thread_;
  // Event file descriptor used to stop async thread
  int event_fd_;
};

}  // namespace isaac
}  // namespace nvidia
