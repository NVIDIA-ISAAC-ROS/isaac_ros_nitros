/*
Copyright (c) 2023, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#ifndef NVIDIA_ISAAC_ROS_EXTENSIONS_MESSAGE_RELAY_HPP_
#define NVIDIA_ISAAC_ROS_EXTENSIONS_MESSAGE_RELAY_HPP_

#include <condition_variable>
#include <mutex>
#include <vector>

#include "gxf/core/component.hpp"
#include "gxf/std/codelet.hpp"
#include "gxf/std/receiver.hpp"

namespace nvidia {
namespace isaac_ros {

// Receives messages, stores them and provides thread-safe access to them.
class MessageRelay : public gxf::Codelet {
 public:
  gxf_result_t registerInterface(gxf::Registrar* registrar) override;
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

  gxf::Parameter<gxf::Handle<gxf::Receiver>> source_;
  gxf::Parameter<uint64_t> max_waiting_count_;
  gxf::Parameter<bool> drop_waiting_;
  gxf::Parameter<int64_t> callback_address_;

  std::vector<gxf::Entity> entities_waiting_;
  std::vector<gxf::Entity> entities_in_vault_;

  std::mutex mutex_;
  std::condition_variable condition_variable_;
  bool alive_;

  std::function<void(void)> * callback_{nullptr};
};

}  // namespace isaac_ros
}  // namespace nvidia

#endif  // NVIDIA_ISAAC_ROS_EXTENSIONS_MESSAGE_RELAY_HPP_
