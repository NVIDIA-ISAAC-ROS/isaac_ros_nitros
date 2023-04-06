/*
Copyright (c) 2018-2022, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#if ((defined(_MSVC_LANG) && _MSVC_LANG >= 201703L) || __cplusplus >= 201703L)

#include <optional>

#else

#include <experimental/optional>

namespace std {
using experimental::optional;
using experimental::nullopt;
using experimental::make_optional;
}  // namespace std

#endif
