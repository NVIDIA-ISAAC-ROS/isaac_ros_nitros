/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include <dlfcn.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#include "gxf/core/entity.hpp"
#include "gxf/std/timestamp.hpp"
#pragma GCC diagnostic pop

#include "isaac_ros_nitros/nitros_context.hpp"
#include "rclcpp/rclcpp.hpp"


namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

gxf_context_t NitrosContext::shared_context_ = nullptr;
std::mutex NitrosContext::shared_context_mutex_;
std::set<std::string> NitrosContext::loaded_extension_filenames_;

std::vector<std::string> SplitStrings(const std::string & list_of_files)
{
  std::vector<std::string> filenames;

  for (size_t begin = 0;; ) {
    const size_t end = list_of_files.find(',', begin);
    if (end == std::string::npos) {
      if (begin == 0 && !list_of_files.empty()) {
        filenames.push_back(list_of_files);
      } else if (!list_of_files.substr(begin).empty()) {
        filenames.push_back(list_of_files.substr(begin));
      }
      break;
    } else {
      filenames.push_back(list_of_files.substr(begin, end - begin));
      begin = end + 1;
    }
  }

  return filenames;
}

std::vector<char *> ToCStringArray(const std::vector<std::string> & strings)
{
  std::vector<char *> cstrings;
  cstrings.reserve(strings.size());
  for (size_t i = 0; i < strings.size(); ++i) {
    cstrings.push_back(const_cast<char *>(strings[i].c_str()));
  }
  return cstrings;
}

NitrosContext::NitrosContext()
{
  // Mutex: shared_context_mutex_
  const std::lock_guard<std::mutex> lock(NitrosContext::shared_context_mutex_);
  gxf_result_t code;
  if (NitrosContext::shared_context_ == nullptr) {
    code = GxfContextCreate(&context_);
    if (code != GXF_SUCCESS) {
      RCLCPP_ERROR(
        get_logger(),
        "[NitrosContext] GxfContextCreate Error: %s", GxfResultStr(code));
      return;
    }
    code = GxfGetSharedContext(context_, &NitrosContext::shared_context_);
    if (code != GXF_SUCCESS) {
      RCLCPP_ERROR(
        get_logger(),
        "[NitrosContext] GxfGetSharedContext Error: %s", GxfResultStr(code));
      return;
    }
    RCLCPP_INFO(
      get_logger(),
      "[NitrosContext] Creating a new shared context");
  } else {
    code = GxfContextCreate1(NitrosContext::shared_context_, &context_);
    if (code != GXF_SUCCESS) {
      RCLCPP_ERROR(
        get_logger(),
        "[NitrosContext] GxfContextCreate1 Error: %s", GxfResultStr(code));
      return;
    }
    RCLCPP_DEBUG(
      get_logger(),
      "[NitrosContext] Creating a local context based on an existing shared context");
  }

  // Set log severity level for core GXF
  code = GxfSetSeverity(context_, extension_log_severity_);
  if (code != GXF_SUCCESS) {
    RCLCPP_ERROR(
      get_logger(),
      "[NitrosContext] GxfSetSeverity Error: %s", GxfResultStr(code));
    return;
  }
  // End Mutex: shared_context_mutex_
}

void NitrosContext::setGraphNamespace(const std::string & graph_namespace)
{
  graph_namespace_ = graph_namespace;
}

void NitrosContext::setNode(const rclcpp::Node * node)
{
  node_ = node;
}

gxf_context_t NitrosContext::getContext()
{
  return context_;
}

gxf_result_t NitrosContext::getComponentPointer(
  const std::string & entity_name,
  const std::string & component_name,
  const std::string & component_type,
  void ** pointer)
{
  // Mutex: shared_context_mutex_
  const std::lock_guard<std::mutex> lock(NitrosContext::shared_context_mutex_);

  gxf_result_t code;

  gxf_uid_t cid;
  code = getCid(entity_name, component_name, component_type, cid);
  if (code != GXF_SUCCESS) {
    RCLCPP_ERROR(
      get_logger(),
      "[NitrosContext] getCid Error: %s", GxfResultStr(code));
    return code;
  }

  gxf_tid_t tid;
  code = GxfComponentTypeId(context_, component_type.c_str(), &tid);
  if (code != GXF_SUCCESS) {
    RCLCPP_ERROR(
      get_logger(),
      "[NitrosContext] GxfComponentTypeId Error: %s", GxfResultStr(code));
    return code;
  }

  code = GxfComponentPointer(context_, cid, tid, pointer);
  if (code != GXF_SUCCESS) {
    RCLCPP_ERROR(
      get_logger(),
      "[NitrosContext] GxfComponentPointer Error: %s", GxfResultStr(code));
    return code;
  }

  return GXF_SUCCESS;
  // End Mutex: shared_context_mutex_
}

gxf_result_t NitrosContext::getCid(
  const std::string & entity_name,
  const std::string & component_type,
  gxf_uid_t & cid)
{
  return getCid(entity_name, "", component_type, cid);
}

gxf_result_t NitrosContext::getCid(
  const std::string & entity_name,
  const std::string & component_name,
  const std::string & component_type,
  gxf_uid_t & cid)
{
  gxf_result_t code;

  gxf_uid_t eid;
  code = GxfEntityFind(context_, getNamespacedEntityName(entity_name).c_str(), &eid);
  if (code != GXF_SUCCESS) {
    RCLCPP_ERROR(
      get_logger(),
      "[NitrosContext] GXFEntityFind Error: %s", GxfResultStr(code));
    return code;
  }

  gxf_tid_t tid;
  code = GxfComponentTypeId(context_, component_type.c_str(), &tid);
  if (code != GXF_SUCCESS) {
    RCLCPP_ERROR(
      get_logger(),
      "[NitrosContext] GxfComponentTypeId Error: %s", GxfResultStr(code));
    return code;
  }

  if (component_name.empty()) {
    code = GxfComponentFind(context_, eid, tid, nullptr, nullptr, &cid);
  } else {
    code = GxfComponentFind(context_, eid, tid, component_name.c_str(), nullptr, &cid);
  }
  if (code != GXF_SUCCESS) {
    RCLCPP_ERROR(
      get_logger(),
      "[NitrosContext] GxfComponentFind Error: %s", GxfResultStr(code));
    return code;
  }

  return GXF_SUCCESS;
}

// Override a parameter value in the graph to be loaded
void NitrosContext::preLoadGraphSetParameter(
  const std::string & entity_name,
  const std::string & component_name,
  const std::string & parameter_name,
  const std::string & value
)
{
  graph_param_override_string_list_.push_back(
    graph_namespace_ + "_" + entity_name +
    "=" + component_name +
    "=" + parameter_name +
    "=" + value);
}

gxf_result_t NitrosContext::loadExtension(
  const std::string & base_dir,
  const std::string & extension)
{
  // Mutex: shared_context_mutex_
  const std::lock_guard<std::mutex> lock(NitrosContext::shared_context_mutex_);

  // As the underlying context is shared across multiple NitrosNodes, we only need to load
  // those extensions that have not been loaded. Loading same extensions twice will throw an
  // error and GxfLoadExtensions() will stop.
  gxf_result_t code;
  if (NitrosContext::loaded_extension_filenames_.count(extension) > 0) {
    // This extension has been loaded before
    return GXF_SUCCESS;
  }
  RCLCPP_INFO(
    get_logger(),
    "[NitrosContext] Loading extension: %s", extension.c_str());

  // Set the log severity level for the extension
  std::string extension_file_path = base_dir + "/" + extension;
  void * handle = dlopen(extension_file_path.c_str(), RTLD_LAZY);
  if (!handle) {
    RCLCPP_ERROR(
      get_logger(),
      "[NitrosContext] dlopen failed when opening \"%s\": %s",
      extension_file_path.c_str(), dlerror());
    return GXF_FAILURE;
  }

  using func_gxf_set_severity_t = void (*)(nvidia::Severity);
  func_gxf_set_severity_t func_gxf_set_severity =
    reinterpret_cast<func_gxf_set_severity_t>(dlsym(
      handle,
      "_ZN6nvidia11SetSeverityENS_8SeverityE"));
  if (!func_gxf_set_severity) {
    RCLCPP_WARN(
      get_logger(),
      "[NitrosContext] dlopen error when opening \"%s\" to set the log severity level: %s",
      extension_file_path.c_str(), dlerror());
  } else {
    func_gxf_set_severity(static_cast<nvidia::Severity>(extension_log_severity_));
  }

  // Actually load the extension
  NitrosContext::loaded_extension_filenames_.insert(extension);
  const char * extension_array[] = {extension.c_str()};
  const GxfLoadExtensionsInfo load_extension_info{
    extension_array, 1, nullptr, 0, base_dir.c_str()};

  code = GxfLoadExtensions(context_, &load_extension_info);
  if ((code != GXF_SUCCESS) && (code != GXF_FACTORY_DUPLICATE_TID)) {
    RCLCPP_ERROR(
      get_logger(),
      "[NitrosContext] GxfLoadExtensions Error: %s", GxfResultStr(code));
    return code;
  }
  return GXF_SUCCESS;
  // End Mutex: shared_context_mutex_
}

gxf_result_t NitrosContext::loadExtensions(
  const std::string & base_dir,
  const std::vector<std::string> & extensions)
{
  gxf_result_t code;
  for (const std::string & extension : extensions) {
    code = loadExtension(base_dir, extension);
    if (code != GXF_SUCCESS) {
      RCLCPP_ERROR(
        get_logger(),
        "[NitrosContext] loadExtension Error: %s", GxfResultStr(code));
      return code;
    }
  }
  return GXF_SUCCESS;
}

// Loads application graph file(s)
gxf_result_t NitrosContext::loadApplication(const std::string & list_of_files)
{
  // Mutex: shared_context_mutex_
  const std::lock_guard<std::mutex> lock(NitrosContext::shared_context_mutex_);

  const auto filenames = SplitStrings(list_of_files);

  if (filenames.empty()) {
    RCLCPP_ERROR(
      get_logger(),
      "[NitrosContext] An NitrosNode application file has to be specified");
    return GXF_FILE_NOT_FOUND;
  }

  std::vector<char *> param_override_cstring = ToCStringArray(graph_param_override_string_list_);
  for (const auto & filename : filenames) {
    RCLCPP_INFO(get_logger(), "[NitrosContext] Loading application: '%s'", filename.c_str());
    const gxf_result_t code = GxfGraphLoadFile(
      context_,
      filename.c_str(),
      (const char **) &param_override_cstring[0],
      graph_param_override_string_list_.size());
    if (code != GXF_SUCCESS) {return code;}
  }

  return GXF_SUCCESS;
  // End Mutex: shared_context_mutex_
}

gxf_result_t NitrosContext::runGraphAsync()
{
  // Mutex: shared_context_mutex_
  const std::lock_guard<std::mutex> lock(NitrosContext::shared_context_mutex_);

  gxf_result_t code;

  RCLCPP_INFO(get_logger(), "[NitrosContext] Initializing application...");
  code = GxfGraphActivate(context_);
  if (code != GXF_SUCCESS) {
    RCLCPP_ERROR(
      get_logger(),
      "[NitrosContext] GxfGraphActivate Error: %s", GxfResultStr(code));
    return code;
  }

  RCLCPP_INFO(get_logger(), "[NitrosContext] Running application...");
  code = GxfGraphRunAsync(context_);
  if (code != GXF_SUCCESS) {
    RCLCPP_ERROR(
      get_logger(),
      "[NitrosContext] GxfGraphRunAsync Error: %s", GxfResultStr(code));
    return code;
  }

  return GXF_SUCCESS;
  // End Mutex: shared_context_mutex_
}

gxf_result_t NitrosContext::setParameterInt64(
  const std::string & entity_name,
  const std::string & codelet_type,
  const std::string & parameter_name,
  const int64_t parameter_value)
{
  // Mutex: shared_context_mutex_
  const std::lock_guard<std::mutex> lock(NitrosContext::shared_context_mutex_);

  gxf_result_t code;
  gxf_uid_t cid;
  code = getCid(entity_name, codelet_type, cid);
  if (code != GXF_SUCCESS) {
    RCLCPP_ERROR(
      get_logger(),
      "[NitrosContext] Failed to get CID for setting parameters");
    return code;
  }
  code = GxfParameterSetInt64(context_, cid, parameter_name.c_str(), parameter_value);
  if (code != GXF_SUCCESS) {
    RCLCPP_ERROR(
      get_logger(),
      "[NitrosContext] GxfParameterSetInt64 Error: %s", GxfResultStr(code));
    return code;
  }
  return GXF_SUCCESS;
  // End Mutex: shared_context_mutex_
}

gxf_result_t NitrosContext::setParameterUInt64(
  const std::string & entity_name,
  const std::string & codelet_type,
  const std::string & parameter_name,
  const uint64_t parameter_value)
{
  // Mutex: shared_context_mutex_
  const std::lock_guard<std::mutex> lock(NitrosContext::shared_context_mutex_);

  gxf_result_t code;
  gxf_uid_t cid;
  code = getCid(entity_name, codelet_type, cid);
  if (code != GXF_SUCCESS) {
    RCLCPP_ERROR(
      get_logger(),
      "[NitrosContext] Failed to get CID for setting parameters");
    return code;
  }
  code = GxfParameterSetUInt64(context_, cid, parameter_name.c_str(), parameter_value);
  if (code != GXF_SUCCESS) {
    RCLCPP_ERROR(
      get_logger(),
      "[NitrosContext] setParameterUInt64 Error: %s", GxfResultStr(code));
    return code;
  }
  return GXF_SUCCESS;
  // End Mutex: shared_context_mutex_
}

gxf_result_t NitrosContext::setParameterInt32(
  const std::string & entity_name,
  const std::string & codelet_type,
  const std::string & parameter_name,
  const int32_t parameter_value)
{
  // Mutex: shared_context_mutex_
  const std::lock_guard<std::mutex> lock(NitrosContext::shared_context_mutex_);

  gxf_result_t code;
  gxf_uid_t cid;
  code = getCid(entity_name, codelet_type, cid);
  if (code != GXF_SUCCESS) {
    RCLCPP_ERROR(
      get_logger(),
      "[NitrosContext] Failed to get CID for setting parameters");
    return code;
  }
  code = GxfParameterSetInt32(context_, cid, parameter_name.c_str(), parameter_value);
  if (code != GXF_SUCCESS) {
    RCLCPP_ERROR(
      get_logger(),
      "[NitrosContext] GxfParameterSetInt32 Error: %s", GxfResultStr(code));
    return code;
  }
  return GXF_SUCCESS;
  // End Mutex: shared_context_mutex_
}

gxf_result_t NitrosContext::setParameterUInt32(
  const std::string & entity_name,
  const std::string & codelet_type,
  const std::string & parameter_name,
  const uint32_t parameter_value)
{
  // Mutex: shared_context_mutex_
  const std::lock_guard<std::mutex> lock(NitrosContext::shared_context_mutex_);

  gxf_result_t code;
  gxf_uid_t cid;
  code = getCid(entity_name, codelet_type, cid);
  if (code != GXF_SUCCESS) {
    RCLCPP_ERROR(
      get_logger(),
      "[NitrosContext] Failed to get CID for setting parameters");
    return code;
  }
  code = GxfParameterSetUInt32(context_, cid, parameter_name.c_str(), parameter_value);
  if (code != GXF_SUCCESS) {
    RCLCPP_ERROR(
      get_logger(),
      "[NitrosContext] GxfParameterSetUInt32 Error: %s", GxfResultStr(code));
    return code;
  }
  return GXF_SUCCESS;
  // End Mutex: shared_context_mutex_
}

gxf_result_t NitrosContext::setParameterUInt16(
  const std::string & entity_name,
  const std::string & codelet_type,
  const std::string & parameter_name,
  const uint16_t parameter_value)
{
  // Mutex: shared_context_mutex_
  const std::lock_guard<std::mutex> lock(NitrosContext::shared_context_mutex_);

  gxf_result_t code;
  gxf_uid_t cid;
  code = getCid(entity_name, codelet_type, cid);
  if (code != GXF_SUCCESS) {
    RCLCPP_ERROR(
      get_logger(),
      "[NitrosContext] Failed to get CID for setting parameters");
    return code;
  }
  code = GxfParameterSetUInt16(context_, cid, parameter_name.c_str(), parameter_value);
  if (code != GXF_SUCCESS) {
    RCLCPP_ERROR(
      get_logger(),
      "[NitrosContext] GxfParameterSetUInt16 Error: %s", GxfResultStr(code));
    return code;
  }
  return GXF_SUCCESS;
  // End Mutex: shared_context_mutex_
}

gxf_result_t NitrosContext::setParameterFloat64(
  const std::string & entity_name,
  const std::string & codelet_type,
  const std::string & parameter_name,
  const double parameter_value)
{
  // Mutex: shared_context_mutex_
  const std::lock_guard<std::mutex> lock(NitrosContext::shared_context_mutex_);

  gxf_result_t code;
  gxf_uid_t cid;
  code = getCid(entity_name, codelet_type, cid);
  if (code != GXF_SUCCESS) {
    RCLCPP_ERROR(
      get_logger(),
      "[NitrosContext] Failed to get CID for setting parameters");
    return code;
  }
  code = GxfParameterSetFloat64(context_, cid, parameter_name.c_str(), parameter_value);
  if (code != GXF_SUCCESS) {
    RCLCPP_ERROR(
      get_logger(),
      "[NitrosContext] GxfParameterSetFloat64 Error: %s", GxfResultStr(code));
    return code;
  }
  return GXF_SUCCESS;
  // End Mutex: shared_context_mutex_
}

gxf_result_t NitrosContext::setParameterStr(
  const std::string & entity_name,
  const std::string & codelet_type,
  const std::string & parameter_name,
  const std::string & parameter_value)
{
  // Mutex: shared_context_mutex_
  const std::lock_guard<std::mutex> lock(NitrosContext::shared_context_mutex_);

  gxf_result_t code;
  gxf_uid_t cid;
  code = getCid(entity_name, codelet_type, cid);
  if (code != GXF_SUCCESS) {
    RCLCPP_ERROR(
      get_logger(),
      "[NitrosContext] Failed to get CID for setting parameters");
    return code;
  }
  code = GxfParameterSetStr(context_, cid, parameter_name.c_str(), parameter_value.c_str());
  if (code != GXF_SUCCESS) {
    RCLCPP_ERROR(
      get_logger(),
      "[NitrosContext] GxfParameterSetStr Error: %s", GxfResultStr(code));
    return code;
  }
  return GXF_SUCCESS;
  // End Mutex: shared_context_mutex_
}

gxf_result_t NitrosContext::setParameterHandle(
  const std::string & entity_name,
  const std::string & codelet_type,
  const std::string & parameter_name,
  const gxf_uid_t & uid)
{
  // Mutex: shared_context_mutex_
  const std::lock_guard<std::mutex> lock(NitrosContext::shared_context_mutex_);

  gxf_result_t code;
  gxf_uid_t cid;
  code = getCid(entity_name, codelet_type, cid);
  if (code != GXF_SUCCESS) {
    RCLCPP_ERROR(
      get_logger(),
      "[NitrosContext] Failed to get CID for setting parameters");
    return code;
  }
  code = GxfParameterSetHandle(context_, cid, parameter_name.c_str(), uid);
  if (code != GXF_SUCCESS) {
    RCLCPP_ERROR(
      get_logger(),
      "[NitrosContext] GxfParameterSetHandle Error: %s", GxfResultStr(code));
    return code;
  }
  return GXF_SUCCESS;
  // End Mutex: shared_context_mutex_
}

gxf_result_t NitrosContext::setParameterBool(
  const std::string & entity_name,
  const std::string & codelet_type,
  const std::string & parameter_name,
  const bool parameter_value)
{
  // Mutex: shared_context_mutex_
  const std::lock_guard<std::mutex> lock(NitrosContext::shared_context_mutex_);

  gxf_result_t code;
  gxf_uid_t cid;
  code = getCid(entity_name, codelet_type, cid);
  if (code != GXF_SUCCESS) {
    RCLCPP_ERROR(
      get_logger(),
      "[NitrosContext] Failed to get CID for setting parameters");
    return code;
  }
  code = GxfParameterSetBool(context_, cid, parameter_name.c_str(), parameter_value);
  if (code != GXF_SUCCESS) {
    RCLCPP_ERROR(
      get_logger(),
      "[NitrosContext] GxfParameterSetBool Error: %s", GxfResultStr(code));
    return code;
  }
  return GXF_SUCCESS;
  // End Mutex: shared_context_mutex_
}

gxf_result_t NitrosContext::setParameter1DStrVector(
  const std::string & entity_name,
  const std::string & codelet_type,
  const std::string & parameter_name,
  const std::vector<std::string> & parameter_value)
{
  // Mutex: shared_context_mutex_
  const std::lock_guard<std::mutex> lock(NitrosContext::shared_context_mutex_);

  gxf_result_t code;
  gxf_uid_t cid;
  code = getCid(entity_name, codelet_type, cid);
  if (code != GXF_SUCCESS) {
    RCLCPP_ERROR(
      get_logger(),
      "[NitrosContext] Failed to get CID for setting parameters");
    return code;
  }

  std::vector<char *> parameter_value_cstring = ToCStringArray(parameter_value);

  code = GxfParameterSet1DStrVector(
    context_, cid, parameter_name.c_str(),
    (const char **) &parameter_value_cstring[0], parameter_value_cstring.size());
  if (code != GXF_SUCCESS) {
    RCLCPP_ERROR(
      get_logger(),
      "[NitrosContext] GxfParameterSet1DStrVector Error: %s", GxfResultStr(code));
    return code;
  }
  return GXF_SUCCESS;
  // End Mutex: shared_context_mutex_
}

gxf_result_t NitrosContext::setParameter1DInt32Vector(
  const std::string & entity_name,
  const std::string & codelet_type,
  const std::string & parameter_name,
  std::vector<int32_t> & parameter_value)
{
  // Mutex: shared_context_mutex_
  const std::lock_guard<std::mutex> lock(NitrosContext::shared_context_mutex_);

  gxf_result_t code;
  gxf_uid_t cid;
  code = getCid(entity_name, codelet_type, cid);
  if (code != GXF_SUCCESS) {
    RCLCPP_ERROR(
      get_logger(),
      "[NitrosContext] Failed to get CID for setting parameters");
    return code;
  }

  code = GxfParameterSet1DInt32Vector(
    context_, cid, parameter_name.c_str(),
    parameter_value.data(), parameter_value.size());
  if (code != GXF_SUCCESS) {
    RCLCPP_ERROR(
      get_logger(),
      "[NitrosContext] GxfParameterSet1DInt32Vector Error: %s", GxfResultStr(code));
    return code;
  }
  return GXF_SUCCESS;
  // End Mutex: shared_context_mutex_
}

gxf_result_t NitrosContext::setParameter1DInt64Vector(
  const std::string & entity_name,
  const std::string & codelet_type,
  const std::string & parameter_name,
  std::vector<int64_t> & parameter_value)
{
  // Mutex: shared_context_mutex_
  const std::lock_guard<std::mutex> lock(NitrosContext::shared_context_mutex_);

  gxf_result_t code;
  gxf_uid_t cid;
  code = getCid(entity_name, codelet_type, cid);
  if (code != GXF_SUCCESS) {
    RCLCPP_ERROR(
      get_logger(),
      "[NitrosContext] Failed to get CID for setting parameters");
    return code;
  }

  code = GxfParameterSet1DInt64Vector(
    context_, cid, parameter_name.c_str(),
    parameter_value.data(), parameter_value.size());
  if (code != GXF_SUCCESS) {
    RCLCPP_ERROR(
      get_logger(),
      "[NitrosContext] GxfParameterSet1DInt64Vector Error: %s", GxfResultStr(code));
    return code;
  }
  return GXF_SUCCESS;
  // End Mutex: shared_context_mutex_
}

gxf_result_t NitrosContext::setParameter1DFloat64Vector(
  const std::string & entity_name,
  const std::string & codelet_type,
  const std::string & parameter_name,
  std::vector<double> & parameter_value)
{
  // Mutex: shared_context_mutex_
  const std::lock_guard<std::mutex> lock(NitrosContext::shared_context_mutex_);

  gxf_result_t code;
  gxf_uid_t cid;
  code = getCid(entity_name, codelet_type, cid);
  if (code != GXF_SUCCESS) {
    RCLCPP_ERROR(
      get_logger(),
      "[NitrosContext] Failed to get CID for setting parameters");
    return code;
  }

  code = GxfParameterSet1DFloat64Vector(
    context_, cid, parameter_name.c_str(),
    parameter_value.data(), parameter_value.size());
  if (code != GXF_SUCCESS) {
    RCLCPP_ERROR(
      get_logger(),
      "[NitrosContext] GxfParameterSet1DFloat64Vector Error: %s", GxfResultStr(code));
    return code;
  }
  return GXF_SUCCESS;
  // End Mutex: shared_context_mutex_
}

std::string NitrosContext::getNamespacedEntityName(const std::string & entity_name)
{
  if (graph_namespace_.empty()) {
    return entity_name;
  }
  return graph_namespace_ + "_" + entity_name;
}

rclcpp::Logger NitrosContext::get_logger()
{
  if (node_ != nullptr) {
    return node_->get_logger();
  }
  return rclcpp::get_logger("NitrosContext");
}

gxf_result_t NitrosContext::getEntityTimestamp(
  const gxf_uid_t eid,
  std_msgs::msg::Header & ros_header)
{
  nvidia::gxf::Expected<nvidia::gxf::Entity> entity = nvidia::gxf::Entity::Shared(context_, eid);

  nvidia::gxf::Expected<nvidia::gxf::Handle<nvidia::gxf::Timestamp>> timestamp_handle =
    entity->get<nvidia::gxf::Timestamp>("timestamp");
  if (!timestamp_handle) {
    timestamp_handle = entity->get<nvidia::gxf::Timestamp>();
  }

  if (!timestamp_handle) {
    RCLCPP_ERROR(
      get_logger(),
      "[NitrosContext] Failed to get a timestamp component from an entity: %s",
      GxfResultStr(timestamp_handle.error()));
    return timestamp_handle.error();
  }

  ros_header.stamp.sec =
    static_cast<int32_t>(timestamp_handle.value()->acqtime / static_cast<uint64_t>(1e9));
  ros_header.stamp.nanosec =
    static_cast<uint32_t>(timestamp_handle.value()->acqtime % static_cast<uint64_t>(1e9));

  return GXF_SUCCESS;
}

void NitrosContext::setExtensionLogSeverity(gxf_severity_t severity_level)
{
  extension_log_severity_ = severity_level;
}

gxf_result_t NitrosContext::destroy()
{
  // Mutex: shared_context_mutex_
  const std::lock_guard<std::mutex> lock(NitrosContext::shared_context_mutex_);

  gxf_result_t code;
  RCLCPP_INFO(get_logger(), "[NitrosContext] Interrupting GXF...");
  code = GxfGraphInterrupt(context_);
  if (code != GXF_SUCCESS) {
    RCLCPP_ERROR(get_logger(), "[NitrosContext] GxfGraphInterrupt Error: %s", GxfResultStr(code));
  }

  RCLCPP_INFO(get_logger(), "[NitrosContext] Waiting on GXF...");
  code = GxfGraphWait(context_);
  if (code != GXF_SUCCESS) {
    RCLCPP_ERROR(get_logger(), "[NitrosContext] GxfGraphWait Error: %s", GxfResultStr(code));
    return code;
  }

  RCLCPP_INFO(get_logger(), "[NitrosContext] Deinitializing...");
  code = GxfGraphDeactivate(context_);
  if (code != GXF_SUCCESS) {
    RCLCPP_ERROR(get_logger(), "[NitrosContext] GxfGraphDeactivate Error: %s", GxfResultStr(code));
    return code;
  }

  RCLCPP_INFO(get_logger(), "[NitrosContext] Destroying context");
  code = GxfContextDestroy(context_);
  if (code != GXF_SUCCESS) {
    RCLCPP_ERROR(get_logger(), "[NitrosContext] GxfContextDestroy Error: %s", GxfResultStr(code));
    return code;
  }

  return GXF_SUCCESS;
  // End Mutex: shared_context_mutex_
}

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia
