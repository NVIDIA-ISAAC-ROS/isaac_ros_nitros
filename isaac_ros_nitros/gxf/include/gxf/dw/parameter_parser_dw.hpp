/*
Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#ifndef NVIDIA_GXF_DW_PARAMETER_PARSER_DW_HPP_
#define NVIDIA_GXF_DW_PARAMETER_PARSER_DW_HPP_

#include <algorithm>
#include <cctype>
#include <cstring>
#include <string>

#include "gxf/dw/pod/field.hpp"
#include "gxf/dw/pod_info.hpp"
#include "gxf/std/parameter_parser.hpp"

namespace nvidia {
namespace gxf {

// Custom parameter parser for NumericField
template <typename T>
struct ParameterParser<pod::NumericField<T>> {
  static Expected<pod::NumericField<T>> Parse(gxf_context_t context, gxf_uid_t component_uid,
                                              const char* key, const YAML::Node& node,
                                              const std::string& prefix) {
    return ParameterParser<T>::Parse(context, component_uid, key, node, prefix)
        .map([&](T value) {
          pod::FieldNameBuffer name;
          return name.copy(key, strnlen(key, name.capacity()))
              .substitute_error(GXF_EXCEEDING_PREALLOCATED_SIZE)
              .substitute(pod::NumericField<T>(name, value));
        });
  }
};

// Custom parameter parser for BooleanField
template <>
struct ParameterParser<pod::BooleanField> {
  static Expected<pod::BooleanField> Parse(gxf_context_t context, gxf_uid_t component_uid,
                                           const char* key, const YAML::Node& node,
                                           const std::string& prefix) {
    return ParameterParser<bool>::Parse(context, component_uid, key, node, prefix)
        .map([&](bool value) {
          pod::FieldNameBuffer name;
          return name.copy(key, strnlen(key, name.capacity()))
              .substitute_error(GXF_EXCEEDING_PREALLOCATED_SIZE)
              .substitute(pod::BooleanField(name, value));
        });
  }
};

// Custom parameter parser for StringField
template <size_t N>
struct ParameterParser<pod::StringField<FixedString<N>>> {
  static Expected<pod::StringField<FixedString<N>>> Parse(gxf_context_t context,
                                                          gxf_uid_t component_uid,
                                                          const char* key,
                                                          const YAML::Node& node,
                                                          const std::string& prefix) {
    return ParameterParser<FixedString<N>>::Parse(context, component_uid, key, node, prefix)
        .map([&](FixedString<N> value) {
          pod::FieldNameBuffer name;
          return name.copy(key, strnlen(key, name.capacity()))
              .substitute_error(GXF_EXCEEDING_PREALLOCATED_SIZE)
              .substitute(pod::StringField<FixedString<N>>(name, value));
        });
  }
};

// Custom parameter parser for PodInfo
template <>
struct ParameterParser<PodInfo> {
  template <typename T>
  static Expected<void> ParseOptional(gxf_context_t context, gxf_uid_t component_uid,
                                      const char* key, const YAML::Node& node,
                                      const std::string& prefix, T& value) {
    if (context == kNullContext || component_uid == kNullUid || key == nullptr) {
      return Unexpected{GXF_ARGUMENT_NULL};
    }
    if (!node.IsMap()) {
      return Unexpected{GXF_PARAMETER_PARSER_ERROR};
    }
    const auto entry = node[key];
    if (entry) {
      auto result = ParameterParser<T>::Parse(context, component_uid, key, entry, prefix)
          .assign_to(value);
      if (!result) {
        return ForwardError(result);
      }
    }
    return Success;
  }

  static Expected<PodInfo> Parse(gxf_context_t context, gxf_uid_t component_uid,
                                 const char* key, const YAML::Node& node,
                                 const std::string& prefix) {
    if (context == kNullContext || component_uid == kNullUid || key == nullptr) {
      return Unexpected{GXF_ARGUMENT_NULL};
    }
    if (!node.IsMap()) {
      return Unexpected{GXF_PARAMETER_PARSER_ERROR};
    }

    // Parse 'state'
    constexpr char kKeyState[] = "state";
    const auto node_state = node[kKeyState];
    if (!node_state) {
      GXF_LOG_ERROR("Could not find '%s' in '%s' parameter", kKeyState, key);
      return Unexpected{GXF_PARAMETER_PARSER_ERROR};
    }

    // Convert string to lowercase account for input variability
    std::string value = node_state.as<std::string>();
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c){ return std::tolower(c); });

    PodInfo info;
    if (value == "unknown") {
      info.state = PodInfo::State::kUnknown;
    } else if (value == "stop") {
      info.state = PodInfo::State::kStop;
    } else if (value == "record") {
      info.state = PodInfo::State::kRecord;
    } else if (value == "error") {
      info.state = PodInfo::State::kError;
    } else {
      return Unexpected{GXF_PARAMETER_PARSER_ERROR};
    }

    // Parse optional members
    auto result = ParseOptional(context, component_uid, "uuid1", node, prefix, info.uuid1)
        & ParseOptional(context, component_uid, "uuid2", node, prefix, info.uuid2)
        & ParseOptional(context, component_uid, "timestamp", node, prefix, info.timestamp)
        & ParseOptional(context, component_uid, "duration", node, prefix, info.duration)
        & ParseOptional(context, component_uid, "vin", node, prefix, info.vin)
        & ParseOptional(context, component_uid, "author", node, prefix, info.author)
        & ParseOptional(context, component_uid, "title", node, prefix, info.title)
        & ParseOptional(context, component_uid, "description", node, prefix, info.description)
        & ParseOptional(context, component_uid, "tags", node, prefix, info.tags);

    return result.substitute(info);
  }
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_DW_PARAMETER_PARSER_DW_HPP_
