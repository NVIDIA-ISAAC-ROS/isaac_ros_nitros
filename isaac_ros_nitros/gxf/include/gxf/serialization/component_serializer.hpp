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
#ifndef NVIDIA_GXF_SERIALIZATION_COMPONENT_SERIALIZER_HPP_
#define NVIDIA_GXF_SERIALIZATION_COMPONENT_SERIALIZER_HPP_

#include <functional>
#include <shared_mutex>  // NOLINT
#include <unordered_map>

#include "gxf/core/component.hpp"
#include "gxf/serialization/endpoint.hpp"
#include "gxf/serialization/tid_hash.hpp"

namespace nvidia {
namespace gxf {

// Interface for serializing components
class ComponentSerializer : public Component {
 public:
  virtual ~ComponentSerializer() = default;

  // Checks if the serializer supports the given component type
  // Searches serializer map by default
  virtual gxf_result_t is_supported_abi(gxf_tid_t tid) {
    return getSerializer(tid) ? GXF_SUCCESS : GXF_FAILURE;
  }
  // Serializes component and writes to endpoint
  // Returns the size of the serialized component in bytes
  // Uses functions in serializer map by default
  virtual gxf_result_t serialize_component_abi(gxf_uid_t cid, Endpoint* endpoint, uint64_t* size);
  // Reads from endpoint and deserializes component
  // Uses functions in serializer map by default
  virtual gxf_result_t deserialize_component_abi(gxf_uid_t cid, Endpoint* endpoint);

  // C++ API wrappers
  bool isSupported(gxf_tid_t tid) { return is_supported_abi(tid) == GXF_SUCCESS; }
  Expected<size_t> serializeComponent(UntypedHandle component, Endpoint* endpoint);
  Expected<void> deserializeComponent(UntypedHandle component, Endpoint* endpoint);

 protected:
  // Serializer function handle
  // Takes a component pointer and an endpoint pointer as input
  // Returns the size of the serialized component in bytes
  using Serializer = std::function<Expected<size_t>(void*, Endpoint*)>;
  // Deserializer function handle
  // Takes a component pointer and an endpoint pointer as input
  using Deserializer = std::function<Expected<void>(void*, Endpoint*)>;

  // Returns a serializer for the given type ID
  Expected<Serializer> getSerializer(gxf_tid_t tid) const;
  // Returns a deserializer for the given type ID
  Expected<Deserializer> getDeserializer(gxf_tid_t tid) const;
  // Adds a serializer for the given type ID
  Expected<void> setSerializer(gxf_tid_t tid, Serializer serializer);
  // Adds a deserializer for the given type ID
  Expected<void> setDeserializer(gxf_tid_t tid, Deserializer deserializer);
  // Removes a serializer for the given type ID
  Expected<void> clearSerializer(gxf_tid_t tid) { return setSerializer(tid, nullptr); }
  // Removes a deserializer for the given type ID
  Expected<void> clearDeserializer(gxf_tid_t tid) { return setDeserializer(tid, nullptr); }

  // Returns a serializer for the given type
  template <typename T>
  Expected<Serializer> getSerializer() const {
    gxf_tid_t tid;
    return ExpectedOrCode(GxfComponentTypeId(context(), TypenameAsString<T>(), &tid))
        .and_then([&](){ return getSerializer(tid); });
  }
  // Returns a deserializer for the given type
  template <typename T>
  Expected<Deserializer> getDeserializer() const {
    gxf_tid_t tid;
    return ExpectedOrCode(GxfComponentTypeId(context(), TypenameAsString<T>(), &tid))
        .and_then([&](){ return getDeserializer(tid); });
  }
  // Adds a serializer for the given type
  template <typename T>
  Expected<void> setSerializer(Serializer serializer) {
    gxf_tid_t tid;
    return ExpectedOrCode(GxfComponentTypeId(context(), TypenameAsString<T>(), &tid))
        .and_then([&](){ return setSerializer(tid, serializer); });
  }
  // Adds a deserializer for the given type
  template <typename T>
  Expected<void> setDeserializer(Deserializer deserializer) {
    gxf_tid_t tid;
    return ExpectedOrCode(GxfComponentTypeId(context(), TypenameAsString<T>(), &tid))
        .and_then([&](){ return setDeserializer(tid, deserializer); });
  }
  // Removes a serializer for the given type
  template <typename T>
  Expected<void> clearSerializer(Serializer serializer) {
    gxf_tid_t tid;
    return ExpectedOrCode(GxfComponentTypeId(context(), TypenameAsString<T>(), &tid))
        .and_then([&](){ return clearSerializer(tid); });
  }
  // Removes a deserializer for the given type
  template <typename T>
  Expected<void> clearDeserializer(Deserializer deserializer) {
    gxf_tid_t tid;
    return ExpectedOrCode(GxfComponentTypeId(context(), TypenameAsString<T>(), &tid))
        .and_then([&](){ return clearDeserializer(tid); });
  }

 private:
  // Structure for organizing serialize-deserialize function pairs
  struct SerializerFunctions {
    Serializer serializer;
    Deserializer deserializer;
  };

  // Table that maps component type ID to serializer functions
  std::unordered_map<gxf_tid_t, SerializerFunctions, TidHash> serializer_map_;
  // Mutex to guard concurrent access to serializer map
  mutable std::shared_timed_mutex mutex_;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_SERIALIZATION_COMPONENT_SERIALIZER_HPP_
