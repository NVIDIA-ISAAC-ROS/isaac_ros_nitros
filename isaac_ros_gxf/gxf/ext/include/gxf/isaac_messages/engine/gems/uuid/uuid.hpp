// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2018-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#pragma once

#include <cstdint>
#include <string>

namespace nvidia {
namespace isaac {

// A universally unique identifier (UUID)
class Uuid {
 public:
  static constexpr int kNumBytes = 16;

  // Generates a new UUID
  static Uuid Generate();
  // Creates a UUID from a string. If the string has "unparsed" UUID format it will be parsed as a
  // UUID string. Otherwise it will be treated as an ASCII string. An unparsed UUID string has the
  // format "XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX" with XXX alpha-numeric characters. May assert
  // if neither form of parsing succeeds, e.g. if the string is too long.
  static Uuid FromString(const std::string& str);
  // Parses a string as an ASCII string. Will assert if the string is too long.
  static Uuid FromAsciiString(const std::string& str);
  // Creates a UUID from a serialized UUID string. This will assert in case the string does not
  // have the right format.
  static Uuid FromUuidString(const std::string& str);
  // Creates UUID from two 64-bit integers
  static Uuid FromUInt64(uint64_t lower, uint64_t upper) {
    Uuid uuid;
    uuid.longs_[0] = lower;
    uuid.longs_[1] = upper;
    uuid.is_uuid_ = true;  // FIXME
    return uuid;
  }

  // Creates an invalid (whatever that means) UUID with all 0
  Uuid() { longs_[0] = 0; longs_[1] = 0; is_uuid_ = false; }

  // Iterator interface
  const unsigned char* begin() const { return bytes_; }
  unsigned char* begin() { return bytes_; }
  const unsigned char* end() const { return bytes_ + kNumBytes; }
  unsigned char* end() { return bytes_ + kNumBytes; }

  uint64_t lower() const { return longs_[0]; }
  uint64_t upper() const { return longs_[1]; }

  // A hash value so that we can use the UUID as a key
  size_t hash() const {
    // Should be way faster than a string-based hash
    return lower() ^ upper();
  }

  // Gets an unparsed string representation for the UUID
  const std::string& str() const {
    if (str_.empty()) {
      unparse();
    }
    return str_;
  }
  // Same as str() but directly returns a null-terminated C-string
  const char* c_str() const {
    return str().c_str();
  }

  // Comparison operators
  friend bool operator==(const Uuid& lhs, const Uuid& rhs) {
    return lhs.lower() == rhs.lower() && lhs.upper() == rhs.upper();
  }
  friend bool operator!=(const Uuid& lhs, const Uuid& rhs) {
    return !(lhs == rhs);
  }
  friend bool operator<(const Uuid& lhs, const Uuid& rhs) {
    return lhs.upper() < rhs.upper()
        || (lhs.upper() == rhs.upper() && lhs.lower() < rhs.lower());
  }

 private:
  // Use both 8-bit unsigned and bytes representations
  union {
    unsigned char bytes_[kNumBytes];
    uint64_t longs_[2];
  };

  // True if this is actually a UUID and not a string chosen by the user.
  bool is_uuid_;

  // Cached string version of uuid used for printing
  mutable std::string str_;

  // Computes the unparsed string version of the UUID
  void unparse() const;
};

}  // namespace isaac
}  // namespace nvidia

namespace std {

// Overload for std::hash to be compatible with standard containers
template <>
struct hash<nvidia::isaac::Uuid> {
  size_t operator()(const nvidia::isaac::Uuid& x) const {
    return x.hash();
  }
};

}  // namespace std
