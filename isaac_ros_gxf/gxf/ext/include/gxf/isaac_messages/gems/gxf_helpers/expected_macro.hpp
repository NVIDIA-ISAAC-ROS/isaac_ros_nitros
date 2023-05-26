/*
Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>

#include "gems/gxf_helpers/common_expected_macro.hpp"
#include "gxf/core/expected.hpp"

// This customizes the expected macro, s.t. it can be used with gxf_result_t.
namespace nvidia {
template <>
struct ExpectedMacroConfig<gxf_result_t> {
  constexpr static gxf_result_t DefaultSuccess() { return GXF_SUCCESS; }
  constexpr static gxf_result_t DefaultError() { return GXF_FAILURE; }
  static std::string Name(gxf_result_t result) { return GxfResultStr(result); }
};

// For back-compatibility we define an alias to the original name of the macro when it was gxf
// specific.
#define GXF_RETURN_IF_ERROR RETURN_IF_ERROR
#define GXF_UNWRAP_OR_RETURN UNWRAP_OR_RETURN

}  // namespace nvidia
