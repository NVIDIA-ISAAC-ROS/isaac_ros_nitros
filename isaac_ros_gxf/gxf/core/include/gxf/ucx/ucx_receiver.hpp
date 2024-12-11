// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <ucp/api/ucp.h>
#include <deque>
#include <list>
#include <memory>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "gxf/std/gems/staging_queue/staging_queue.hpp"
#include "gxf/std/receiver.hpp"
#include "gxf/std/resources.hpp"

#include "ucx_common.hpp"
#include "ucx_serialization_buffer.hpp"

#ifndef NVIDIA_GXF_UCX_UCX_RECEIVER_HPP_
#define NVIDIA_GXF_UCX_UCX_RECEIVER_HPP_

namespace nvidia {
namespace gxf {


// Codelet that functions as a receiver on a UCX connection
class UcxReceiver : public Receiver {
 public:
  using queue_t = ::gxf::staging_queue::StagingQueue<Entity>;
  gxf_result_t registerInterface(Registrar* registrar) override;
  gxf_result_t initialize() override;
  gxf_result_t deinitialize() override;
  gxf_result_t init_context(ucp_worker_h  ucp_worker,
                            ucx_am_data_desc* am_data_desc, int fd,
                            bool cpu_data_only, bool enable_async);

  gxf_result_t pop_abi(gxf_uid_t* uid) override;

  gxf_result_t push_abi(gxf_uid_t other) override;

  gxf_result_t peek_abi(gxf_uid_t* uid, int32_t index) override;

  gxf_result_t peek_back_abi(gxf_uid_t* uid, int32_t index) override;

  size_t capacity_abi() override;

  size_t size_abi() override;

  gxf_result_t receive_abi(gxf_uid_t* uid) override;

  size_t back_size_abi() override;

  gxf_result_t sync_abi() override;

  gxf_result_t sync_io_abi() override;

  gxf_result_t wait_abi() override;
  const char* get_addr();

  int get_port();

  Expected<void> set_serialization_buffer(Handle<UcxSerializationBuffer> buffer);

  Expected<void> set_port(int port);

  Parameter<uint64_t> capacity_;
  Parameter<uint64_t> policy_;
  Parameter<std::string> address_;
  Parameter<uint32_t> port_;
  Resource<Handle<GPUDevice>> gpu_device_;
  Parameter<Handle<UcxSerializationBuffer>> buffer_;

 private:
  gxf_result_t receive_message();

  /// @brief finalize request when enable_async_ == false
  gxf_result_t request_finalize_sync(ucp_worker_h ucp_worker, test_req_t* request,
                                     test_req_t* ctx);

  ucp_worker_h     ucp_worker_;
  ucx_am_data_desc* am_data_desc_;
  std::unique_ptr<queue_t> queue_;
  int32_t dev_id_ = 0;
  int efd_signal_;
  bool cpu_data_only_ = false;
  std::list<std::pair<void*, test_req_t*>> requests;
  int enable_async_ = true;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_EXTENSIONS_UCX_UCX_RECEIVER_HPP
