// SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES.
//                         All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//
// NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
// property and proprietary rights in and to this material, related
// documentation and any modifications thereto. Any use, reproduction,
// disclosure or distribution of this material and related documentation
// without an express license agreement from NVIDIA CORPORATION or
// its affiliates is strictly prohibited.

//! @file
//! @brief ANSI escape sequence definitions for formatting console text
//!
//! See, for example,
//! https://en.wikipedia.org/wiki/ANSI_escape_code#SGR_(Select_Graphic_Rendition)_parameters

#pragma once

namespace cumotion {
namespace text_style {

// Reset all attributes.
static constexpr char RESET[] = "\x1B[0m";

// Formatting
static constexpr char BOLD[] = "\x1B[1m";
static constexpr char DIM[] = "\x1B[2m";
static constexpr char UNDERLINE[] = "\x1B[4m";
static constexpr char BLINK[] = "\x1B[5m";
static constexpr char REVERSE[] = "\x1B[7m";

// Foreground colors
static constexpr char DEFAULT[] = "\x1B[39m";
static constexpr char BLACK[] = "\x1B[30m";
static constexpr char RED[] = "\x1B[31m";
static constexpr char GREEN[] = "\x1B[32m";
static constexpr char YELLOW[] = "\x1B[33m";
static constexpr char BLUE[] = "\x1B[34m";
static constexpr char MAGENTA[] = "\x1B[35m";
static constexpr char CYAN[] = "\x1B[36m";
static constexpr char LIGHT_GRAY[] = "\x1B[37m";
static constexpr char DARK_GRAY[] = "\x1B[90m";
static constexpr char BRIGHT_RED[] = "\x1B[91m";
static constexpr char BRIGHT_GREEN[] = "\x1B[92m";
static constexpr char BRIGHT_YELLOW[] = "\x1B[93m";
static constexpr char BRIGHT_BLUE[] = "\x1B[94m";
static constexpr char BRIGHT_MAGENTA[] = "\x1B[95m";
static constexpr char BRIGHT_CYAN[] = "\x1B[96m";
static constexpr char WHITE[] = "\x1B[97m";

// Background colors
static constexpr char BG_DEFAULT[] = "\x1B[49m";
static constexpr char BG_BLACK[] = "\x1B[40m";
static constexpr char BG_RED[] = "\x1B[41m";
static constexpr char BG_GREEN[] = "\x1B[42m";
static constexpr char BG_YELLOW[] = "\x1B[43m";
static constexpr char BG_BLUE[] = "\x1B[44m";
static constexpr char BG_MAGENTA[] = "\x1B[45m";
static constexpr char BG_CYAN[] = "\x1B[46m";
static constexpr char BG_LIGHT_GRAY[] = "\x1B[47m";
static constexpr char BG_DARK_GRAY[] = "\x1B[100m";
static constexpr char BG_BRIGHT_RED[] = "\x1B[101m";
static constexpr char BG_BRIGHT_GREEN[] = "\x1B[102m";
static constexpr char BG_BRIGHT_YELLOW[] = "\x1B[103m";
static constexpr char BG_BRIGHT_BLUE[] = "\x1B[104m";
static constexpr char BG_BRIGHT_MAGENTA[] = "\x1B[105m";
static constexpr char BG_BRIGHT_CYAN[] = "\x1B[106m";
static constexpr char BG_WHITE[] = "\x1B[107m";

}  // namespace text_style
}  // namespace cumotion
