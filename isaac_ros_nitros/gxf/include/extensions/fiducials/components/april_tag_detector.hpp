/*
Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#ifndef NVIDIA_ISAAC_EXTENSIONS_FIDUCIALS_COMPONENTS_APRIL_TAG_DETECTOR_HPP_
#define NVIDIA_ISAAC_EXTENSIONS_FIDUCIALS_COMPONENTS_APRIL_TAG_DETECTOR_HPP_

#include <memory>
#include <string>

#include "extensions/messages/camera_image_message.hpp"
#include "gxf/std/allocator.hpp"
#include "gxf/std/codelet.hpp"
#include "gxf/std/receiver.hpp"
#include "gxf/std/transmitter.hpp"

namespace nvidia {
namespace isaac {

// Takes an image as input and detects and decodes any AprilTags found
// in the image. The AprilTags codelet uses AprilTag 3 reference implementation
// from AprilRobotics. This implementation is known to be slower than nvAprilTag
// CUDA-optimized implementation. It is provided as a reference implementation only
// without any performance optimizations.
class AprilTagDetector : public gxf::Codelet {
 public:
  // Explicitly declare constructors and destructors
  // to get around forward declaration of AprilTagsData
  AprilTagDetector();
  ~AprilTagDetector();

  gxf_result_t registerInterface(gxf::Registrar* registrar) override;
  gxf_result_t initialize() override;
  gxf_result_t deinitialize() override;

  gxf_result_t start() override { return GXF_SUCCESS; }
  gxf_result_t tick() override;
  gxf_result_t stop() override;

 private:
  // Initializes AprilTag detector with camera intrinsics
  void createAprilTagDetector(const ::isaac::geometry::PinholeD& intrinsics_info);

  gxf::Parameter<gxf::Handle<gxf::Receiver>> camera_image_;
  gxf::Parameter<gxf::Handle<gxf::Transmitter>> april_tags_;
  gxf::Parameter<gxf::Handle<gxf::Allocator>> allocator_;
  gxf::Parameter<double> tag_dimensions_;
  gxf::Parameter<std::string> tag_family_;

  // Hide the AprilTagData implementation details
  struct AprilTagData;
  std::unique_ptr<AprilTagData> impl_;
};

}  // namespace isaac
}  // namespace nvidia

#endif  // NVIDIA_ISAAC_EXTENSIONS_FIDUCIALS_COMPONENTS_APRIL_TAG_DETECTOR_HPP_
