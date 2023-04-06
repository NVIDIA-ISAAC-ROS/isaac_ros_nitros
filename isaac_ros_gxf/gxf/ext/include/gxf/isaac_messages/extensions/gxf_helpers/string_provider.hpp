/*
Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#ifndef NVIDIA_ISAAC_EXTENSIONS_GXF_HELPERS_STRING_PROVIDER_HPP_
#define NVIDIA_ISAAC_EXTENSIONS_GXF_HELPERS_STRING_PROVIDER_HPP_

#include <string>

#include "gxf/core/component.hpp"

namespace nvidia {
namespace isaac {

// A component that provides access to a string. This is useful for situations where one would like
// to share the same string between multiple components, for example to implement a namespace/prefix
// for a frame name of a pose tree frame.
class StringProvider : public gxf::Component {
 public:
  gxf_result_t registerInterface(gxf::Registrar* registrar) override;

  // Gets the value of the string.
  std::string value() const;

 private:
  gxf::Parameter<gxf::Handle<StringProvider>> prefix_;
  gxf::Parameter<gxf::Handle<StringProvider>> suffix_;
  gxf::Parameter<std::string> value_;
};

}  // namespace isaac
}  // namespace nvidia

#endif  // NVIDIA_ISAAC_EXTENSIONS_GXF_HELPERS_STRING_PROVIDER_HPP_
