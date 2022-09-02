/*
Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#ifndef NVIDIA_ISAAC_EXTENSIONS_FIDUCIALS_FIDUCIAL_WRITER_HPP_
#define NVIDIA_ISAAC_EXTENSIONS_FIDUCIALS_FIDUCIAL_WRITER_HPP_

#include "engine/gems/serialization/json.hpp"
#include "gxf/serialization/endpoint.hpp"
#include "gxf/std/codelet.hpp"
#include "gxf/std/receiver.hpp"

namespace nvidia {
namespace isaac {

// Writes fiducials to an endpoint in JSON format
class FiducialWriter : public gxf::Codelet {
 public:
  gxf_result_t registerInterface(gxf::Registrar* registrar) override;
  gxf_result_t initialize() override { return GXF_SUCCESS; }
  gxf_result_t deinitialize() override { return GXF_SUCCESS; }

  gxf_result_t start() override;
  gxf_result_t tick() override;
  gxf_result_t stop() override;

 private:
  gxf::Parameter<gxf::Handle<gxf::Receiver>> fiducial_list_;
  gxf::Parameter<gxf::Handle<gxf::Endpoint>> endpoint_;

  // JSON object
  ::isaac::Json json_;
};

}  // namespace isaac
}  // namespace nvidia

#endif  // NVIDIA_ISAAC_EXTENSIONS_FIDUCIALS_FIDUCIAL_WRITER_HPP_
