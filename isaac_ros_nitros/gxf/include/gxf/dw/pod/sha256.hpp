/*
Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#ifndef NVIDIA_GXF_DW_POD_SHA256_HPP_
#define NVIDIA_GXF_DW_POD_SHA256_HPP_

#include <array>

#include "common/fixed_string.hpp"
#include "common/span.hpp"
#include "gxf/core/expected.hpp"

namespace nvidia {
namespace gxf {
namespace pod {

/// The SHA256 class Generates the SHA256 sum of a given string
/// Adapted from https://www.programmingalgorithms.com/algorithm/sha256
class SHA256 {
 public:
  static constexpr size_t HASH_LENGTH = 32;
  using Result = std::array<byte, HASH_LENGTH>;
  using String = FixedString<HASH_LENGTH * 2>;

  static Expected<Result> Hash(const Span<const byte> data);
  static Expected<String> Hash(const Span<const char> data);

  SHA256() : context_{}, hash_{} { Initialize(context_); }

  /// Reset hasher to be fed again
  void reset() { Initialize(context_); }

  /// Hash a given array
  Expected<void> hashData(const Span<const byte> data) { return Update(context_, data); }

  /// Finalize computation of the hash, i.e. make solution available through `hash()`
  Expected<void> finalize() {
    return Finalize(context_)
        .map([&](Result result) {
          hash_ = result;
          return Success;
        });
  }

  /// Return hashed result
  const Result& hash() const { return hash_; }
  /// Return base64 encoding of the hash
  Expected<String> toString() const { return ToString(hash_); }

 private:
  struct SHA256_CTX {
    uint8_t data[64];
    uint32_t datalen;
    uint32_t bitlen[2];
    uint32_t state[8];
  };

  static void Initialize(SHA256_CTX& ctx);
  static Expected<void> Update(SHA256_CTX& ctx, const Span<const byte> data);
  static Expected<void> Transform(SHA256_CTX& ctx, const Span<const byte> data);
  static Expected<Result> Finalize(SHA256_CTX& ctx);
  static Expected<String> ToString(const Result& hash);

  SHA256_CTX context_;
  Result hash_;
};

}  // namespace pod
}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_DW_POD_SHA256_HPP_
