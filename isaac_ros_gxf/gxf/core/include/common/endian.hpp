/*
 * SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef NVIDIA_COMMON_ENDIAN_HPP_
#define NVIDIA_COMMON_ENDIAN_HPP_

#ifdef _QNX_SOURCE
#include <net/netbyte.h>
#else
#include <endian.h>
#endif

#include "common/type_utils.hpp"

namespace nvidia {

/// Returns true if the machine stores multi-byte integers in little-endian format
inline bool IsLittleEndian() {
  const uint16_t test = 0x0102;
  return *reinterpret_cast<const uint8_t*>(&test) == 0x02;
}

/// Convert @a value to little-endian.
/// This overload should only be used if @c T is a signed integer type.
template <typename T, typename = EnableIf_t<IsIntegral_v<T>>>
inline T EncodeLittleEndian(T value) {
  static_assert(IsSigned_v<T>);
  using U = MakeUnsigned_t<T>;
  return static_cast<T>(EncodeLittleEndian(static_cast<U>(value)));
}

template <> inline uint64_t EncodeLittleEndian<uint64_t>(uint64_t value) { return htole64(value); }
template <> inline uint32_t EncodeLittleEndian<uint32_t>(uint32_t value) { return htole32(value); }
template <> inline uint16_t EncodeLittleEndian<uint16_t>(uint16_t value) { return htole16(value); }
template <> inline uint8_t  EncodeLittleEndian<uint8_t> (uint8_t  value) { return value; }

/// Convert @a value from little-endian.
/// This overload should only be used if @c T is a signed integer type.
template <typename T, typename = EnableIf_t<IsIntegral_v<T>>>
inline T DecodeLittleEndian(T value) {
  static_assert(IsSigned_v<T>);
  using U = MakeUnsigned_t<T>;
  return static_cast<T>(DecodeLittleEndian(static_cast<U>(value)));
}

template <> inline uint64_t DecodeLittleEndian<uint64_t>(uint64_t value) { return le64toh(value); }
template <> inline uint32_t DecodeLittleEndian<uint32_t>(uint32_t value) { return le32toh(value); }
template <> inline uint16_t DecodeLittleEndian<uint16_t>(uint16_t value) { return le16toh(value); }
template <> inline uint8_t  DecodeLittleEndian<uint8_t> (uint8_t  value) { return value; }

/// Returns true if the machine stores multi-byte integers in big-endian format
inline bool IsBigEndian() {
  const uint16_t test = 0x0102;
  return *reinterpret_cast<const uint8_t*>(&test) == 0x01;
}

/// Convert @a value to big-endian.
/// This overload should only be used if @c T is a signed integer type.
template <typename T, typename = EnableIf_t<IsIntegral_v<T>>>
inline T EncodeBigEndian(T value) {
  static_assert(IsSigned_v<T>);
  using U = MakeUnsigned_t<T>;
  return static_cast<T>(EncodeBigEndian(static_cast<U>(value)));
}

template <> inline uint64_t EncodeBigEndian<uint64_t>(uint64_t value) { return htobe64(value); }
template <> inline uint32_t EncodeBigEndian<uint32_t>(uint32_t value) { return htobe32(value); }
template <> inline uint16_t EncodeBigEndian<uint16_t>(uint16_t value) { return htobe16(value); }
template <> inline uint8_t  EncodeBigEndian<uint8_t> (uint8_t  value) { return value; }

/// Convert @a value from big-endian.
/// This overload should only be used if @c T is a signed integer type.
template <typename T, typename = EnableIf_t<IsIntegral_v<T>>>
inline T DecodeBigEndian(T value) {
  static_assert(IsSigned_v<T>);
  using U = MakeUnsigned_t<T>;
  return static_cast<T>(DecodeBigEndian(static_cast<U>(value)));
}

template <> inline uint64_t DecodeBigEndian<uint64_t>(uint64_t value) { return be64toh(value); }
template <> inline uint32_t DecodeBigEndian<uint32_t>(uint32_t value) { return be32toh(value); }
template <> inline uint16_t DecodeBigEndian<uint16_t>(uint16_t value) { return be16toh(value); }
template <> inline uint8_t  DecodeBigEndian<uint8_t> (uint8_t  value) { return value; }

}  // namespace nvidia

#endif  // NVIDIA_COMMON_ENDIAN_HPP_
