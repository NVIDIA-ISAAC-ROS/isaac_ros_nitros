/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#ifndef NVIDIA_GXF_STD_UNBOUNDED_ALLOCATOR_HPP_
#define NVIDIA_GXF_STD_UNBOUNDED_ALLOCATOR_HPP_

#include <mutex>
#include <set>

#include "gxf/std/allocator.hpp"

namespace nvidia {
namespace gxf {

// An allocater which uses cudaMalloc/cudaMallocHost dynamically without a pool. Does not provide
// bounded execution times.
class UnboundedAllocator : public Allocator {
 public:
  UnboundedAllocator() = default;
  ~UnboundedAllocator() = default;

  gxf_result_t registerInterface(Registrar* registrar) override;
  gxf_result_t initialize() override;
  gxf_result_t deinitialize() override;

  gxf_result_t is_available_abi(uint64_t size) override;
  gxf_result_t allocate_abi(uint64_t size, int32_t storage_type, void** pointer) override;
  gxf_result_t free_abi(void* pointer) override;

 private:
  Parameter<bool> do_not_use_cuda_malloc_host_;

  // Mutex to protect cuda_blocks_
  std::mutex mutex_;
  // Remember the blocks so that we know how to delete them
  std::set<void*> cuda_blocks_;
  std::set<void*> cuda_host_blocks_;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_STD_UNBOUNDED_ALLOCATOR_HPP_
