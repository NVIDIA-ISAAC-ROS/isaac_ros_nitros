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

#ifndef NVIDIA_GXF_STD_ASYNC_BUFFER_TRANSMITTER_HPP_
#define NVIDIA_GXF_STD_ASYNC_BUFFER_TRANSMITTER_HPP_

#include <memory>

#include "gxf/core/component.hpp"
#include "gxf/core/entity.hpp"
#include "gxf/core/handle.hpp"
#include "gxf/std/transmitter.hpp"

namespace nvidia {
namespace gxf {

/**
 * @brief A transmitter which uses a Simpson's four-slot buffer to enable lockless and asynchronous
 * communication. This transmitter is designed to be used in a single producer and single consumer
 * scenario.
 *
 */
class AsyncBufferTransmitter : public Transmitter {
 public:
  gxf_result_t initialize() override;

  gxf_result_t pop_abi(gxf_uid_t* uid) override;

  gxf_result_t pop_io_abi(gxf_uid_t* uid) override;

  gxf_result_t push_abi(gxf_uid_t other) override;

  gxf_result_t peek_abi(gxf_uid_t* uid, int32_t index) override;

  size_t capacity_abi() override;

  size_t size_abi() override;

  gxf_result_t publish_abi(gxf_uid_t uid) override;

  size_t back_size_abi() override;

  gxf_result_t sync_abi() override;

  gxf_result_t sync_io_abi() override;

 private:
  Entity entity_;
  int size_;
};

}  // namespace gxf
}  // namespace nvidia

#endif /* NVIDIA_GXF_STD_ASYNC_BUFFER_TRANSMITTER_HPP_ */
