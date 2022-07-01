/*
Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#ifndef NVIDIA_GXF_DW_TESTS_POD_INFO_COMPARATOR_HPP_
#define NVIDIA_GXF_DW_TESTS_POD_INFO_COMPARATOR_HPP_

#include <vector>

#include "gxf/dw/pod_info.hpp"
#include "gxf/std/codelet.hpp"
#include "gxf/std/receiver.hpp"

namespace nvidia {
namespace gxf {
namespace test {

// Compares entities with a PodInfo component
class PodInfoComparator : public Codelet {
 public:
  gxf_result_t registerInterface(Registrar* registrar) override;
  gxf_result_t initialize() override;
  gxf_result_t deinitialize() override;

  gxf_result_t start() override { return GXF_SUCCESS; }
  gxf_result_t tick() override;
  gxf_result_t stop() override { return GXF_SUCCESS; }

 private:
  Parameter<Handle<Receiver>> input_;
  Parameter<std::vector<PodInfo>> infos_;

  std::vector<PodInfo> values_;
};

}  // namespace test
}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_DW_TESTS_POD_INFO_COMPARATOR_HPP_
