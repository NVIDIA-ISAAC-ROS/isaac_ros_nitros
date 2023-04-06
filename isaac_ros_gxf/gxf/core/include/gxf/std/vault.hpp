/*
 * SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#ifndef NVIDIA_GXF_STD_VAULT_HPP_
#define NVIDIA_GXF_STD_VAULT_HPP_

#include <condition_variable>
#include <mutex>
#include <vector>

#include "gxf/core/component.hpp"
#include "gxf/std/codelet.hpp"
#include "gxf/std/receiver.hpp"

namespace nvidia {
namespace gxf {

// Receives messages, stores them and provides thread-safe access to them.
class Vault : public Codelet {
 public:
  gxf_result_t registerInterface(Registrar* registrar) override;
  gxf_result_t initialize() override;
  gxf_result_t start() override;
  gxf_result_t tick() override;
  gxf_result_t stop() override;
  gxf_result_t deinitialize() override;

  // Waits until at least the given number of entities have arrived, stores them in the vault,
  // and returns their UIDs.
  std::vector<gxf_uid_t> storeBlocking(size_t count);

  // Waits until at least the given number of entities have arrived, stores them in the vault,
  // and returns their UIDs or times out within a targeted duration (nanoseconds).
  std::vector<gxf_uid_t> storeBlockingFor(size_t count, int64_t duration_ns);

  // Tries to grab at most specified number of entities and return without waiting.
  std::vector<gxf_uid_t> store(size_t max_count);

  // Removes the given entities from the vault
  void free(const std::vector<gxf_uid_t>& entities);

 private:
  // Stores entities assuming lock
  std::vector<gxf_uid_t> storeImpl(size_t max_count);

  Parameter<Handle<Receiver>> source_;
  Parameter<uint64_t> max_waiting_count_;
  Parameter<bool> drop_waiting_;

  std::vector<Entity> entities_waiting_;
  std::vector<Entity> entities_in_vault_;

  std::mutex mutex_;
  std::condition_variable condition_variable_;
  bool alive_;
};

}  // namespace gxf
}  // namespace nvidia

#endif
