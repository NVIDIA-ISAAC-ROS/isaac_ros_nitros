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
#ifndef NVIDIA_GXF_CORE_ENTITY_HPP_
#define NVIDIA_GXF_CORE_ENTITY_HPP_

#include <utility>

#include "common/assert.hpp"
#include "common/fixed_vector.hpp"
#include "common/type_name.hpp"
#include "gxf/core/expected.hpp"
#include "gxf/core/expected_macro.hpp"
#include "gxf/core/gxf.h"
#include "gxf/core/handle.hpp"

namespace nvidia {
namespace gxf {

/**
 * @brief All GXF objects are entities. An entity owns multiple components which define the functionality
 * of the entity. Entities themselves are nothing more than a unique identifier. Entities created using the
 * C++ type is ref counted. The ref count is automatically decreased when the entity object is destructed or
 * goes out of scope.
 *
 */
class Entity {
 public:
  /**
   * @brief Creates a new entity using the given context and optionally set the given name. The
   * caller of this api own's the object. The reference count is set to 1 and it is automatically
   * reduced when this object is destroyed or goes out of scope.
   *
   * @param context A valid GXF context
   * @param name Name for the entity object to be created
   * @return Expected<Entity> Entity object wrapped in an expected type to capture error on failure
   */
  static Expected<Entity> New(gxf_context_t context, const char* name = nullptr) {
    gxf_uid_t eid;
    const GxfEntityCreateInfo info = {0};
    void* item_ptr = nullptr;
    gxf_result_t code = GxfCreateEntityAndGetItem(context, &info, &eid, &item_ptr);
    if (code != GXF_SUCCESS) { return Unexpected{code}; }

    return Shared(context, eid, item_ptr);
  }

  /**
   * @brief Creates an entity handle based on an existing ID and takes ownership.
   * Reference count is not increased.
   *
   * @param context A valid GXF context
   * @param eid The unique object ID (UID) of a valid entity
   * @param item_ptr An optional entity item pointer
   * @return Expected<Entity> Entity object wrapped in an expected type to capture error on failure
   */
  static Expected<Entity> Own(gxf_context_t context, gxf_uid_t eid, void* item_ptr = nullptr) {
    Entity result;
    result.context_ = context;
    result.eid_ = eid;
    result.entity_item_ptr_ = item_ptr;
    return result;
  }

  /**
   * @brief Creates an entity handle based on an existing ID and shares ownership.
   * Reference count is increased by one.
   *
   * @param context  A valid GXF context
   * @param eid The unique object ID (UID) of a valid entity
   * @param item_ptr An optional entity item pointer
   * @return Expected<Entity> Entity object wrapped in an expected type to capture error on failure
   */
  static Expected<Entity> Shared(gxf_context_t context, gxf_uid_t eid, void* ptr = nullptr) {
    Entity result;
    result.context_ = context;
    result.eid_ = eid;
    result.entity_item_ptr_ = ptr;
    const gxf_result_t code = GxfEntityRefCountInc(context, eid);
    if (code != GXF_SUCCESS) {
      return Unexpected{code};
    } else {
      return result;
    }
  }

  /**
   * @brief Construct a new entity object using default constructor.
   * This is a null entity without a valid context of entity ID.
   *
   */
  Entity() = default;

  /**
   * @brief Construct a new entity object by copying from another entity object.
   *
   * @param other A valid Entity object
   */
  Entity(const Entity& other) {
    eid_ = other.eid();
    context_ = other.context();
    entity_item_ptr_ = other.entity_item_ptr();
    if (eid_ != kNullUid) {
      // FIXME(dweikersdorf) How do we deal with failure?
      GxfEntityRefCountInc(context_, eid_);
    }
  }

  /**
   * @brief Construct a new entity object by moving the contents from an existing entity object.
   *
   * @param other A valid Entity object
   */
  Entity(Entity&& other) {
    context_ = other.context_;
    eid_ = other.eid_;
    entity_item_ptr_ = other.entity_item_ptr_;
    other.context_ = kNullContext;
    other.eid_ = kNullUid;
    other.entity_item_ptr_ = kNullUid;
  }

  /**
   * @brief Operator overload for copy assignment operation
   *
   * @param other A valid Entity object
   * @return Entity& The resulting copied entity object
   */
  Entity& operator=(const Entity& other) {
    // In case other point to the same entity, nothing needs to be done.
    if (eid_ == other.eid() && context_ == other.context()) {
      return *this;
    }
    if (eid_ != kNullUid) {
      release();
    }
    context_ = other.context();
    eid_ = other.eid();
    entity_item_ptr_ = other.entity_item_ptr();
    if (eid_ != kNullUid) {
      // FIXME(dweikersdorf) How do we deal with failure?
      GxfEntityRefCountInc(context_, eid_);
    }
    return *this;
  }

  /**
   * @brief Operator overload for move assignment operation
   *
   * @param other A valid Entity object
   * @return Entity& The resulting moved entity object
   */
  Entity& operator=(Entity&& other) {
    // In case other is this, then nothing should be done.
    if (&other == this) {
      return *this;
    }
    if (eid_ != kNullUid) {
      release();
    }
    context_ = other.context_;
    eid_ = other.eid_;
    entity_item_ptr_ = other.entity_item_ptr_;
    other.entity_item_ptr_ = nullptr;
    other.context_ = kNullContext;
    other.eid_ = kNullUid;
    return *this;
  }

  /**
   * @brief Destroy the Entity object. Reduces the reference count by 1.
   *
   */
  ~Entity() {
    if (eid_ != kNullUid) {
      release();
    }
  }

  /**
   * @brief Activates the entity. See GxfEntityActivate in gxf.h
   *
   * @return Expected<void>
   */
  Expected<void> activate() {
    return ExpectedOrCode(GxfEntityActivate(context(), eid()));
  }

  /**
   * @brief Deactivates the entity. See GxfEntityDectivate in gxf.h
   *
   * @return Expected<void>
   */
  Expected<void> deactivate() {
    return ExpectedOrCode(GxfEntityDeactivate(context(), eid()));
  }

  /**
   * @brief Clone an entity from an existing entity object. The returned entity shares the
   * ownership with the entity being cloned from. Reference count is increased by one.
   *
   * @return Expected<Entity>
   */
  Expected<Entity> clone() const {
    return Shared(context(), eid(), entity_item_ptr());
  }

  /**
   * @brief Returns the GXF context of the entity.
   *
   * @return gxf_context_t
   */
  gxf_context_t context() const { return context_; }

  /**
   * @brief Returns the  unique object ID (UID) of the entity
   *
   * @return gxf_uid_t
   */
  gxf_uid_t eid() const { return eid_; }

  /**
   * @brief Checks if an entity object is null (empty) or not
   *
   * @return true If entity is not null (empty) entity object
   * @return false If the entity is a valid object
   */
  bool is_null() const { return eid_ == kNullUid; }

  /**
   * @brief The name of the entity or empty string if no name has been given to the entity.
   *
   * @return const char* pointer to name of the entity
   */
  const char* name() const {
    const char* ptr;
    const gxf_result_t result = GxfEntityGetName(context_, eid_, &ptr);
    return (result == GXF_SUCCESS) ? ptr : "";
  }

  /**
   * @brief Adds a component with given type ID.
   *
   * @param tid A valid type ID of a registered component
   * @param name Name to be given to the newly created component instance
   * @return Expected<UntypedHandle> An untyped handle to component or error on failure
   */
  Expected<UntypedHandle> add(gxf_tid_t tid, const char* name = nullptr) {
    gxf_uid_t cid;
    void* comp_ptr = nullptr;
    GXF_RETURN_IF_ERROR(check_entity_item_ptr());
    const auto result = GxfComponentAddAndGetPtr(context(), entity_item_ptr(), tid,
                                                 name, &cid, &comp_ptr);
    if (result != GXF_SUCCESS) {
      return Unexpected{result};
    }
    return UntypedHandle::Create(context(), cid, tid, comp_ptr);
  }

  /**
   * @brief Removes a component with given type ID.
   *
   * @param tid A valid type ID of a registered component
   * @param name Name to be given to the newly created component instance
   * @return Expected<void> or error on failure
   */
  Expected<void> remove(gxf_tid_t tid, const char* name = nullptr) {
    // TODO(pshekdar): Add new API GxfComponentRemoveCpp
    const auto result = GxfComponentRemove(context(), eid(), tid, name);
    return ExpectedOrCode(result);
  }

  /**
   * @brief Adds a component with given template component type
   *
   * @tparam T A valid template type of a registered component
   * @param name Name to be given to the newly created component instance
   * @return Expected<Handle<T>> Typed Handle to the component instance
   */
  template <typename T>
  Expected<Handle<T>> add(const char* name = nullptr) {
    gxf_tid_t tid;
    const auto result_1 = GxfComponentTypeId(context(), TypenameAsString<T>(), &tid);
    if (result_1 != GXF_SUCCESS) {
      return Unexpected{result_1};
    }
    gxf_uid_t cid;
    void* comp_ptr = nullptr;
    GXF_RETURN_IF_ERROR(check_entity_item_ptr());
    const auto result_2 = GxfComponentAddAndGetPtr(context(), entity_item_ptr(), tid, name, &cid,
                                                   &comp_ptr);
    if (result_2 != GXF_SUCCESS) {
      return Unexpected{result_2};
    }
    return Handle<T>::Create(context(), cid, tid, comp_ptr);
  }

  /**
   * @brief Removes a component with given template component type
   *
   * @tparam T A valid template type of a registered component
   * @param name Name of the component
   * @return Expected<void> or error on failure
   */
  template <typename T>
  Expected<void> remove(const char* name = nullptr) {
    gxf_tid_t tid;
    // TODO(pshekdar): Add new API GxfComponentRemoveCpp
    const auto result_1 = GxfComponentTypeId(context(), TypenameAsString<T>(), &tid);
    if (result_1 != GXF_SUCCESS) {
      return Unexpected{result_1};
    }
    const auto result_2 = GxfComponentRemove(context(), eid(), tid, name);
    return ExpectedOrCode(result_2);
  }

  /**
   * @brief Removes a component with given uid
   *
   * @tparam cid A valid uid of the component
   * @return Expected<void> or error on failure
   */
  Expected<void> remove(gxf_uid_t& cid) {
    const auto result = GxfComponentRemoveWithUID(context(), cid);
    return ExpectedOrCode(result);
  }

  /**
   * @brief Gets a component by type ID. Asserts if no such component.
   *
   * @param tid A valid type ID of a registered component
   * @param name Name of the component to lookup
   * @return Expected<UntypedHandle> An untyped handle to component or error on failure
   */
  Expected<UntypedHandle> get(gxf_tid_t tid, const char* name = nullptr) const {
    gxf_uid_t cid;
    void* comp_ptr = nullptr;
    GXF_RETURN_IF_ERROR(check_entity_item_ptr());
    const auto result = GxfComponentFindAndGetPtr(context(), eid(),
                                                  static_cast<void*>(entity_item_ptr()),
                                                  tid, name, nullptr, &cid, &comp_ptr);
    if (result != GXF_SUCCESS) {
      return Unexpected{result};
    }
    return UntypedHandle::Create(context(), cid, tid, comp_ptr);
  }

  /**
   * @brief Gets a component by type. Asserts if no such component.
   *
   * @tparam T A valid template type of a registered component
   * @param name Name of the component to lookup
   * @return Expected<Handle<T>> Typed Handle to the component instance or error on failure
   */
  template <typename T>
  Expected<Handle<T>> get(const char* name = nullptr) const {
    gxf_tid_t tid;
    GXF_RETURN_IF_ERROR(check_entity_item_ptr());
    const auto result_1 = GxfComponentTypeId(context(), TypenameAsString<T>(), &tid);
    if (result_1 != GXF_SUCCESS) {
      return Unexpected{result_1};
    }
    gxf_uid_t cid;
    void* comp_ptr = nullptr;
    const auto result_2 = GxfComponentFindAndGetPtr(context(), eid(),
                                                    static_cast<void*>(entity_item_ptr()),
                                                    tid, name, nullptr, &cid, &comp_ptr);
    if (result_2 != GXF_SUCCESS) {
      return Unexpected{result_2};
    }
    return Handle<T>::Create(context(), cid, tid, comp_ptr);
  }

  /**
   * @brief Finds all components in an entity. A fixed-size vector of untyped handles of all the
   * components are returned.
   *
   * @tparam N Capacity of the FixedVector
   * @return Expected<FixedVector<UntypedHandle, N>> A fixed-size vector of untyped handles of all
   * the components allocated on stack
   */
  template <size_t N = kMaxComponents>
  Expected<FixedVector<UntypedHandle, N>> findAll() const {
    const gxf_context_t c_context = context();
    const gxf_uid_t c_eid = eid();
    gxf_uid_t cids[N];
    uint64_t num_cids = N;
    const gxf_result_t code = GxfComponentFindAll(c_context, c_eid, &num_cids, cids);
    if (code != GXF_SUCCESS) {
      return Unexpected{code};
    }
    FixedVector<UntypedHandle, N> components;
    for (size_t i = 0; i < num_cids; i++) {
      const auto result = UntypedHandle::Create(c_context, cids[i])
          .map([&](UntypedHandle component) {
            return components.push_back(component)
                .substitute_error(GXF_EXCEEDING_PREALLOCATED_SIZE);
          });
      if (!result) {
        return ForwardError(result);
      }
    }
    return components;
  }

  /**
   * @brief Finds all components of given type.
   *
   * @tparam T A valid template type of a registered component
   * @tparam N Capacity of the FixedVector
   * @return Expected<FixedVector<Handle<T>, N>> A fixed-size vector of typed handles of given
   * component type allocated on stack
   */
  template <typename T, size_t N = kMaxComponents>
  Expected<FixedVector<Handle<T>, N>> findAll() const {
    const gxf_context_t c_context = context();
    const gxf_uid_t c_eid = eid();
    gxf_tid_t tid;
    const gxf_result_t code = GxfComponentTypeId(c_context, TypenameAsString<T>(), &tid);
    if (code != GXF_SUCCESS) {
      return Unexpected{code};
    }
    FixedVector<Handle<T>, N> components;
    for (int offset = 0; static_cast<size_t>(offset) < N; offset++) {
      gxf_uid_t cid;
      const gxf_result_t code = GxfComponentFind(c_context, c_eid, tid, nullptr, &offset, &cid);
      if (code != GXF_SUCCESS) {
        break;
      }
      const auto result = Handle<T>::Create(c_context, cid)
          .map([&](Handle<T> component) {
            return components.push_back(component)
                .substitute_error(GXF_EXCEEDING_PREALLOCATED_SIZE);
          });
      if (!result) {
        return ForwardError(result);
      }
    }
    return components;
  }

  /**
   * @brief Finds all components in an entity. A fixed-size vector of untyped handles of all the
   * components are returned.
   *
   * @tparam N Capacity of the FixedVector
   * @return Expected<FixedVector<UntypedHandle, N>> A fixed-size vector of untyped handles of all
   * the components allocated on heap
   */
  template <size_t N = kMaxComponents>
  Expected<FixedVector<UntypedHandle, N>> findAllHeap() const {
    const gxf_context_t c_context = context();
    const gxf_uid_t c_eid = eid();
    gxf_uid_t cids[N];
    uint64_t num_cids = N;
    const gxf_result_t code = GxfComponentFindAll(c_context, c_eid, &num_cids, cids);
    if (code != GXF_SUCCESS) {
      return Unexpected{code};
    }
    FixedVector<UntypedHandle> components;
    components.reserve(num_cids);
    for (size_t i = 0; i < num_cids; i++) {
      const auto result = UntypedHandle::Create(c_context, cids[i])
          .map([&](UntypedHandle component) {
            return components.push_back(component)
                .substitute_error(GXF_EXCEEDING_PREALLOCATED_SIZE);
          });
      if (!result) {
        return ForwardError(result);
      }
    }
    return components;
  }

  /**
   * @brief Finds all components of given type.
   *
   * @tparam T A valid template type of a registered component
   * @tparam N Capacity of the FixedVector
   * @return Expected<FixedVector<Handle<T>>> A fixed-size vector of typed handles of given
   *  component type allocated on heap
   */
  template <typename T, size_t N = kMaxComponents>
  Expected<FixedVector<Handle<T>>> findAllHeap() const {
    const gxf_context_t c_context = context();
    const gxf_uid_t c_eid = eid();
    gxf_tid_t tid;
    const gxf_result_t code = GxfComponentTypeId(c_context, TypenameAsString<T>(), &tid);
    if (code != GXF_SUCCESS) {
      return Unexpected{code};
    }
    FixedVector<Handle<T>> components;
    components.reserve(N);
    for (int offset = 0; static_cast<size_t>(offset) < N; offset++) {
      gxf_uid_t cid;
      const gxf_result_t code = GxfComponentFind(c_context, c_eid, tid, nullptr, &offset, &cid);
      if (code != GXF_SUCCESS) {
        break;
      }
      const auto result = Handle<T>::Create(c_context, cid)
          .map([&](Handle<T> component) {
            return components.push_back(component)
                .substitute_error(GXF_EXCEEDING_PREALLOCATED_SIZE);
          });
      if (!result) {
        return ForwardError(result);
      }
    }
    return components;
  }

  /**
   * @brief Returns the pointer to Entity Item for the entity
   *
   * @return void * entity item ptr
   */
  void* entity_item_ptr() const {
    check_entity_item_ptr();
    return entity_item_ptr_;
  }

 private:
  /**
   * @brief Set the entity item ptr if its not set already
   *
   * @return Expected<void> Success or error code
   */
  Expected<void> check_entity_item_ptr() const {
    if (entity_item_ptr_ != nullptr) {
      return Success;
    }
    auto code = GxfEntityGetItemPtr(context(), eid(),
                                    reinterpret_cast<void**>(&(entity_item_ptr_)));
    return code == GXF_SUCCESS ? Success : Unexpected(code);
  }

  void release() {
    GxfEntityRefCountDec(context_, eid_);  // TODO(v2) We should use the error code, but we can't
                                           //          do anything about it..
    eid_ = kNullUid;
    entity_item_ptr_ = nullptr;
  }

  gxf_context_t context_ = kNullContext;
  gxf_uid_t eid_ = kNullUid;
  mutable void* entity_item_ptr_ = nullptr;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_CORE_ENTITY_HPP_
