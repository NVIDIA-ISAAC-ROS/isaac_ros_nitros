// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#ifndef NVIDIA_GXF_COMMON_YAML_PARSER_HPP_
#define NVIDIA_GXF_COMMON_YAML_PARSER_HPP_

#include <complex>
#include <regex>
#include <sstream>
#include <string>

#include "common/logger.hpp"
#include "yaml-cpp/yaml.h"


inline static bool is_space(unsigned char c) {
  return c == ' ';
}

/**
 * Custom YAML parser for std::complex types
 *
 * Handles parsing of strings containing a complex floating point value.
 *
 * Examples of valid strings are:
 *    "1.0 + 2.5j"
 *    "-1.0 - 3i"
 *    "1+3.3j"
 *
 * There may be 0 or 1 space between a + or - sign and the digits.
 * Either "i" or "j" must appear immediately after the second number.
 */
template <typename T>
struct YAML::convert<std::complex<T>> {
  static Node encode(const std::complex<T>& data) {
    std::stringstream ss;
    ss << data.real();
    if (data.imag() >= 0) {
      ss << '+';
    }
    ss << data.imag() << 'j';
    Node node = YAML::Load(ss.str());
    return node;
  }

  static bool decode(const Node& node, std::complex<T>& data) {
    if (!node.IsScalar()) {
      GXF_LOG_ERROR("complex<T> decode: expected a scalar");
      return false;
    }
    std::string value = node.as<std::string>();

    std::regex complex_reg("\\s*([+-]?\\s?\\d*\\.?\\d+)\\s?([+-]{1}\\s?\\d*\\.?\\d+)[ij]{1}\\s*$");
    std::smatch m;
    if (std::regex_search(value, m, complex_reg)) {
      if (m.size() != 3) {
        GXF_LOG_ERROR("unexpected match size: [%ld],  matched: [%s]", m.size(), m.str(0).c_str());
      }
      // extract the real and imaginary components of the number
      std::string real_str = m.str(1);
      std::string imag_str = m.str(2);

      // remove any white space around + or - (necessary for std::stod to work)
      real_str.erase(std::remove_if(real_str.begin(), real_str.end(), is_space), real_str.end());
      imag_str.erase(std::remove_if(imag_str.begin(), imag_str.end(), is_space), imag_str.end());

      // format real and imaginary strings as floating point
      double real = std::stod(real_str);
      double imag = std::stod(imag_str);
      data = std::complex<T>(real, imag);
    } else {
      GXF_LOG_ERROR("failed to match expected regex for complex<T>");
      return false;
    }
    return true;
  }
};


// operator overload for std::complex<T>
template <typename T>
YAML::Emitter& operator<<(YAML::Emitter& out, const std::complex<T>& c) {
  std::stringstream ss;
  ss << c.real();
  if (c.imag() >= 0) ss << '+';
  ss << c.imag() << 'j';
  out << ss.str();
  return out;
}

#endif  // NVIDIA_GXF_COMMON_YAML_PARSER_HPP_
