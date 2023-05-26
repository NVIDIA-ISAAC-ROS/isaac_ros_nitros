/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <algorithm>
#include <cctype>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace isaac {

// Checks wether a string starts with a certain string
inline bool StartsWith(const std::string& str, const std::string& prefix) {
  return str.size() >= prefix.size()
      && str.compare(0, prefix.size(), prefix) == 0;
}

// Checks wether a string ends with a certain string
inline bool EndsWith(const std::string& str, const std::string& suffix) {
  return str.size() >= suffix.size()
      && str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
}

// Creates a random alpha-numeric string
template <typename URBG>
std::string RandomAlphaNumeric(size_t length, URBG&& rng) {
  // All alpha numeric characters
  constexpr char kAlphaNumeric[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
      "abcdefghijklmnopqrstuvwxyz"
      "0123456789";
  std::uniform_int_distribution<size_t> random_index(0, sizeof(kAlphaNumeric) - 1);
  std::string result(length, 0);
  std::generate_n(result.begin(), length, [&]() { return kAlphaNumeric[random_index(rng)]; });
  return result;
}

// Splits a string by deliminator character
inline std::vector<std::string> SplitString(const std::string& str, const char delim) {
  std::vector<std::string> result;
  std::stringstream ss(str);
  std::string item;
  while (!std::getline(ss, item, delim).fail()) {
    result.push_back(item);
  }
  return result;
}

// Converts a string to lower-case characters
inline std::string ToLowerCase(std::string data) {
  std::transform(data.begin(), data.end(), data.begin(), ::tolower);
  return data;
}

// Trims the spaces at the beginning and end of string
inline std::string TrimString(std::string s) {
  s.erase(s.begin(), find_if(s.begin(), s.end(), [](int ch) { return !std::isspace(ch); }));
  s.erase(find_if(s.rbegin(), s.rend(), [](int ch) { return !std::isspace(ch); }).base(), s.end());
  return s;
}

// Returns a new string containing the last count characters of a string
inline std::string TakeLast(const std::string& str, size_t count) {
  if (str.length() <= count) {
    return str;
  } else {
    if (count < 2) {
      return str.substr(str.length() - count, count);
    } else {
      return ".." + str.substr(str.length() - count + 2, count - 2);
    }
  }
}

}  // namespace isaac
