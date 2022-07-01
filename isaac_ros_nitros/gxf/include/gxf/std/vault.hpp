/*
Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
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
