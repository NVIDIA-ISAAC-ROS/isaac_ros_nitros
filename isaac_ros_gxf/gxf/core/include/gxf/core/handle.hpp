// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2021-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#ifndef NVIDIA_GXF_CORE_HANDLE_HPP
#define NVIDIA_GXF_CORE_HANDLE_HPP

#include <cinttypes>
#include <string>

#include "common/assert.hpp"
#include "common/type_name.hpp"
#include "gxf/core/expected.hpp"
#include "gxf/core/gxf.h"

namespace nvidia {
namespace gxf {

// A handle which gives access to a component without specifying its type.
class UntypedHandle {
 public:
  UntypedHandle(const UntypedHandle& component) = default;
  UntypedHandle(UntypedHandle&& component) = default;
  UntypedHandle& operator=(const UntypedHandle& component) = default;
  UntypedHandle& operator=(UntypedHandle&& component) = default;

  // Creates a null untyped handle
  static UntypedHandle Null() {
    return UntypedHandle{kNullContext, kNullUid};
  }

  // Creates a new untyped handle
  static Expected<UntypedHandle> Create(gxf_context_t context, gxf_uid_t cid) {
    UntypedHandle untyped_handle{context, cid};
    gxf_tid_t tid;
    gxf_result_t code = GxfComponentType(context, cid, &tid);
    if (code != GXF_SUCCESS) {
      return Unexpected{code};
    }
    const auto result = untyped_handle.initialize(tid);
    if (!result) {
      return ForwardError(result);
    }
    return untyped_handle;
  }

  // Creates a new untyped handle with component pointer
  static Expected<UntypedHandle> Create(gxf_context_t context, gxf_uid_t cid,
                                        gxf_tid_t tid, void* ptr) {
    return UntypedHandle{context, cid, tid, ptr};
  }

  // The context to which the component belongs
  gxf_context_t context() const { return context_; }
  // The ID of the component.
  gxf_uid_t cid() const { return cid_; }
  // The type ID describing the component type.
  gxf_tid_t tid() const { return tid_; }
  // Get the pointer to underlying component object
  void* get_ptr() const {return pointer_;}
  // Returns null if the handle is equivalent to a nullptr.
  bool is_null() const {
    return context_ == kNullContext || cid_ == kNullUid || pointer_ == nullptr;
  }
  // Same as 'is_null'.
  explicit operator bool() const { return !is_null(); }
  // The component name
  const char* name() const {
    const char* result;
    const gxf_result_t code = GxfComponentName(context(), cid(), &result);
    return (code == GXF_SUCCESS) ? result : "";
  }

 protected:
  UntypedHandle(gxf_context_t context, gxf_uid_t cid)
    : context_{context}, cid_{cid}, tid_{GxfTidNull()}, pointer_{nullptr} { }

  UntypedHandle(gxf_context_t context, gxf_uid_t cid, gxf_tid_t tid, void* pointer)
    : context_{context}, cid_{cid}, tid_{tid}, pointer_{pointer} {}

  Expected<void> initialize(gxf_tid_t tid) {
    tid_ = tid;
    if (pointer_ == nullptr) {
        return ExpectedOrCode(GxfComponentPointer(context_, cid_, tid_, &pointer_));
    }
    return Success;
  }

  Expected<void> initialize(const char* type_name) {
    gxf_tid_t tid;
    gxf_result_t code = GxfComponentTypeId(context_, type_name, &tid);
    if (code != GXF_SUCCESS) {
      return Unexpected{code};
    }
    return initialize(tid);
  }

  Expected<void> verifyPointer() const {
    if (pointer_ == nullptr) {
      GXF_LOG_ERROR("Handle pointer is null for component %s - id %ld", name(), cid());
      return Unexpected{GXF_FAILURE};
    }
    void* raw_pointer;
    auto result = GxfComponentPointer(context(), cid(), tid_, &raw_pointer);
    if (result != GXF_SUCCESS) {
      return Unexpected{result};
    }
    if (raw_pointer != pointer_) {
      GXF_LOG_ERROR("Handle pointers do not match for component %s: %p vs %p",
                    name(), raw_pointer, pointer_);
      return Unexpected{GXF_FAILURE};
    }
    return Success;
  }

  gxf_context_t context_;
  gxf_uid_t cid_;
  gxf_tid_t tid_;
  void* pointer_;
};

// A handle which gives access to a component with a specific type.
template <typename T>
class Handle : public UntypedHandle {
 public:
  // Creates a null handle
  static Handle Null() {
    return Handle{};
  }

  // An unspecified handle is a unique handle used to denote a component which
  // will be created in the future. A parameter of Handle to a type does not consider
  // "Unspecified" as a valid parameter value and hence this handle must only be used
  // when defining a graph application across different files and the parameters are set
  // in a delayed fashion (sub-graphs and parameter yaml files for example)
  // Entity activation will fail if any of the mandatory parameters are "Unspecified"
  static Handle Unspecified() {
    return Handle(kNullContext, kUnspecifiedUid);
  }

  // Creates a new handle
  static Expected<Handle> Create(gxf_context_t context, gxf_uid_t cid) {
    Handle handle{context, cid};
    const auto result = handle.initialize(TypenameAsString<T>());
    if (!result) {
      return ForwardError(result);
    }
    return handle;
  }

  static Expected<Handle> Create(gxf_context_t context, gxf_uid_t cid,
                                 gxf_tid_t tid, void * ptr) {
    if (GxfTidIsNull(tid) || ptr == nullptr) {
      return Create(context, cid);
    }
    Handle handle{context, cid, tid, ptr};
    return handle;
  }

  // Creates a new handle from an untyped handle
  static Expected<Handle> Create(const UntypedHandle& untyped_handle) {
    return Create(untyped_handle.context(), untyped_handle.cid(),
                  untyped_handle.tid(), untyped_handle.get_ptr());
  }

  friend bool operator==(const Handle& lhs, const Handle& rhs) {
    return lhs.context() == rhs.context() && lhs.cid() == rhs.cid();
  }

  friend bool operator!=(const Handle& lhs, const Handle& rhs) {
    return lhs.context() != rhs.context() || lhs.cid() != rhs.cid();
  }

  friend bool operator<(const Handle& lhs, const Handle& rhs) {
    return lhs.cid() < rhs.cid();
  }

  Handle(gxf_context_t context = kNullContext, gxf_uid_t uid = kNullUid)
    : UntypedHandle{context, uid} {}

  Handle(gxf_context_t context , gxf_uid_t uid , gxf_tid_t tid, void* ptr)
    : UntypedHandle{context, uid, tid, ptr} {}

  ~Handle() = default;

  Handle(const Handle& component) = default;
  Handle(Handle&& component) = default;
  Handle& operator=(const Handle& component) = default;
  Handle& operator=(Handle&& component) = default;

  template <typename Derived>
  Handle(const Handle<Derived>& derived) : UntypedHandle(derived) {
    static_assert(std::is_base_of<T, Derived>::value,
                  "Handle conversion is only allowed from derived class to base class");
  }

  // Allow conversion from handle to pointer
  operator T*() const { return get(); }

  T* operator->() const {
    return get();
  }

  T* get() const {
    // Verifying the pointer every time the handle is accessed causes a
    // significant bottle neck in perf. This should be removed in a future
    // release. Uncomment below line if there is any problem.
    // GXF_ASSERT(verifyPointer(), "Invalid Component Pointer.");
    GXF_ASSERT_FALSE(pointer_ == nullptr);
    return reinterpret_cast<T*>(pointer_);
  }

  Expected<T*> try_get() const {
    // Verifying the pointer every time the handle is accessed causes a
    // significant bottle neck in perf. This should be removed in a future
    // release. Uncomment below line if there is any problem.
    // if (!verifyPointer()) { return Unexpected{GXF_FAILURE}; }
    if (pointer_ == nullptr) { return Unexpected{GXF_FAILURE}; }
    return reinterpret_cast<T*>(pointer_);
  }

 private:
  using UntypedHandle::UntypedHandle;
};


// Creates a new handle given the string representation of the handle in a yaml blob
// If the value is of format <entity-name>/<component-name> then cid is ignored.
// If the entity name is not specified, the value indicates just the component name, then
// 'cid' is used to query the entity name to create the handle
template <typename T>
static Expected<Handle<T>> CreateHandleFromString(gxf_context_t context, gxf_uid_t cid,
                                                  const char* value) {
  gxf_uid_t eid;
  std::string component_name;
  const std::string tag{value};
  const size_t pos = tag.find('/');

  if (pos == std::string::npos) {
    // Get the entity of this component
    auto result = GxfComponentEntity(context, cid, &eid);
    if (result != GXF_SUCCESS) {
      GXF_LOG_ERROR("Failed to find entity for component [C%05" PRId64 "] with name [%s] with"
                    " error %s", cid, value, GxfResultStr(result));
      return Unexpected{result};
    }
    component_name = tag;
  } else {
    // Split the tag into entity and component name
    const std::string entity_name = tag.substr(0, pos);
    component_name = tag.substr(pos + 1);
    // Search for the entity
    auto result = GxfEntityFind(context, entity_name.c_str(), &eid);
    if (result != GXF_SUCCESS) {
      GXF_LOG_ERROR("Failed to find entity [E%05" PRId64 "] with name [%s] while parsing '%s'",
          eid, entity_name.c_str(), tag.c_str());
      return Unexpected{result};
    }
  }

  // Get the type id of the component we are are looking for.
  gxf_tid_t tid;
  auto result = GxfComponentTypeId(context, TypenameAsString<T>(), &tid);
  if (result != GXF_SUCCESS) {
    GXF_LOG_ERROR("Failed to find type ID of component type [%s] with error %s",
        TypenameAsString<T>(), GxfResultStr(result));
    return Unexpected{result};
  }

  gxf_uid_t cid2;
  // Find the component in the indicated entity
  result = GxfComponentFind(context, eid, tid, component_name.c_str(), nullptr, &cid2);
  if (result != GXF_SUCCESS) {
    GXF_LOG_ERROR("Failed to find component in entity [E%05" PRId64 "] with name [%s]",
        eid, component_name.c_str());
    return Unexpected{result};
  }

  return Handle<T>::Create(context, cid2);
}

}  // namespace gxf
}  // namespace nvidia

#endif
