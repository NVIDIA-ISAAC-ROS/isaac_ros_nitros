/*
 * SPDX-FileCopyrightText: Copyright (c) 2020 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#ifndef DOUBLE_BUFFER_RECEIVER_HPP
#define DOUBLE_BUFFER_RECEIVER_HPP

#include <memory>

#include "gxf/core/component.hpp"
#include "gxf/core/entity.hpp"
#include "gxf/core/handle.hpp"
#include "gxf/std/gems/staging_queue/staging_queue.hpp"
#include "gxf/std/receiver.hpp"

namespace nvidia {
namespace gxf {

// A receiver which uses a double-buffered queue where new messages are first pushed to a
// backstage. Incoming messages are not immediately available and need to be moved to the mainstage
// first.
class DoubleBufferReceiver : public Receiver {
 public:
  using queue_t = ::gxf::staging_queue::StagingQueue<Entity>;

  gxf_result_t registerInterface(Registrar* registrar) override;

  gxf_result_t initialize() override;

  gxf_result_t deinitialize() override;

  gxf_result_t pop_abi(gxf_uid_t* uid) override;

  gxf_result_t push_abi(gxf_uid_t other) override;

  gxf_result_t peek_abi(gxf_uid_t* uid, int32_t index) override;

  gxf_result_t peek_back_abi(gxf_uid_t* uid, int32_t index) override;

  size_t capacity_abi() override;

  size_t size_abi() override;

  gxf_result_t receive_abi(gxf_uid_t* uid) override;

  size_t back_size_abi() override;

  gxf_result_t sync_abi() override;

  Parameter<uint64_t> capacity_;
  Parameter<uint64_t> policy_;

 private:
  std::unique_ptr<queue_t> queue_;
};

}  // namespace gxf
}  // namespace nvidia

#endif
