/*
Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#ifndef NVIDIA_GXF_DW_TESTS_POD_INFO_GENERATOR_HPP_
#define NVIDIA_GXF_DW_TESTS_POD_INFO_GENERATOR_HPP_

#include <vector>

#include "gxf/dw/pod_info.hpp"
#include "gxf/std/codelet.hpp"
#include "gxf/std/scheduling_terms.hpp"
#include "gxf/std/transmitter.hpp"

namespace nvidia {
namespace gxf {
namespace test {

// Generates entities with a PodInfo component
class PodInfoGenerator : public Codelet {
 public:
  gxf_result_t registerInterface(Registrar* registrar) override;
  gxf_result_t initialize() override { return GXF_SUCCESS; }
  gxf_result_t deinitialize() override { return GXF_SUCCESS; }

  gxf_result_t start() override;
  gxf_result_t tick() override;
  gxf_result_t stop() override { return GXF_SUCCESS; }

 private:
  Parameter<Handle<Transmitter>> output_;
  Parameter<Handle<BooleanSchedulingTerm>> boolean_scheduling_term_;
  Parameter<std::vector<PodInfo>> infos_;

  std::vector<PodInfo>::const_iterator iter_;
};

}  // namespace test
}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_DW_TESTS_POD_INFO_GENERATOR_HPP_
