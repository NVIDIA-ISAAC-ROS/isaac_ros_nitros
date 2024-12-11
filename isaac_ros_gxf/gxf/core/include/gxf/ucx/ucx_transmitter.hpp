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

#include <condition_variable>
#include <list>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

#include "ucp/api/ucp.h"

#include "gxf/std/gems/staging_queue/staging_queue.hpp"
#include "gxf/std/resources.hpp"
#include "gxf/std/transmitter.hpp"
#include "ucx_common.hpp"
#include "ucx_entity_serializer.hpp"
#include "ucx_serialization_buffer.hpp"

#ifndef NVIDIA_GXF_UCX_UCX_TRANSMITTER_HPP_
#define NVIDIA_GXF_UCX_UCX_TRANSMITTER_HPP_

namespace nvidia {
namespace gxf {

// Codelet that functions as a transmitter in a UCX connection
class UcxTransmitter : public Transmitter {
 public:
  using queue_t = ::gxf::staging_queue::StagingQueue<Entity>;

  gxf_result_t init_context(ucp_context_h ucp_context,
                            Handle<EntitySerializer> serializer,
                            ucp_worker_h ucp_worker,
                            ucp_ep_h* ep,
                            bool* connection_closed_p,
                            bool reconnect,
                            bool cpu_data_only,
                            bool enable_async,
                            std::list<UcxTransmitterSendContext_>* send_queue,
                            std::condition_variable* cv,
                            std::mutex* mtx);

  gxf_result_t registerInterface(Registrar* registrar) override;

  gxf_result_t initialize() override;

  gxf_result_t deinitialize() override;

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

  Expected<void> set_port(int port);

  Expected<void> set_serialization_buffer(Handle<UcxSerializationBuffer> buffer);

  Parameter<uint64_t> capacity_;
  Parameter<uint64_t> policy_;

 private:
  gxf_result_t send_am(Entity& entity);
  gxf_result_t create_client_connection();
  gxf_result_t create_client_connection_with_retries();
  gxf_result_t check_connection_and_connect();

  ucp_worker_h  ucp_worker_;
  ucp_ep_h* ep_;
  Parameter<std::string> receiver_address_;
  Parameter<std::string> local_address_;
  Parameter<uint32_t> port_;
  Parameter<uint32_t> local_port_;
  Parameter<uint32_t> maximum_connection_retries_;
  Resource<Handle<GPUDevice>> gpu_device_;
  int32_t dev_id_ = 0;
  Parameter<Handle<UcxSerializationBuffer>> buffer_;

  Handle<EntitySerializer> entity_serializer_;
  std::unique_ptr<queue_t> queue_;
  bool* connection_closed_p_;
  bool reconnect_;
  bool cpu_data_only_;
  std::list<UcxTransmitterSendContext_>* send_queue_;
  std::condition_variable* cv_;
  std::mutex* mtx_;
  int index = 0;
  int* id_;
  int enable_async_ = true;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_EXTENSIONS_UCX_UCX_TRANSMITTER_HPP
