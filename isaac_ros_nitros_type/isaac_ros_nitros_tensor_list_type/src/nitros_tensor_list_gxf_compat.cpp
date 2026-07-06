// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifdef NITROS_GXF_COMPAT_MODE

// INTERIM: GXF compatibility for NitrosTensorList (buffer <-> GXF entity for NitrosNode graphs)
#include <memory>
#include <string>
#include <vector>

#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_list.hpp"
#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_list_gxf_compat.hpp"
#include "isaac_ros_nitros_tensor_list_type/nitros_data_type.hpp"
#include "isaac_ros_nitros/types/nitros_buffer.hpp"
#include "isaac_ros_nitros/types/type_adapter_nitros_context.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wpedantic"
#include "gxf/core/entity.hpp"
#include "gxf/core/gxf.h"
#include "gxf/std/tensor.hpp"
#include "gxf/std/timestamp.hpp"
#pragma GCC diagnostic pop

#include "rclcpp/rclcpp.hpp"


namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{
namespace
{

NitrosDataType GxfPrimitiveToNitrosDataType(nvidia::gxf::PrimitiveType pt)
{
  switch (pt) {
    case nvidia::gxf::PrimitiveType::kInt8:
      return NitrosDataType::kInt8;
    case nvidia::gxf::PrimitiveType::kUnsigned8:
      return NitrosDataType::kUnsigned8;
    case nvidia::gxf::PrimitiveType::kInt16:
      return NitrosDataType::kInt16;
    case nvidia::gxf::PrimitiveType::kUnsigned16:
      return NitrosDataType::kUnsigned16;
    case nvidia::gxf::PrimitiveType::kInt32:
      return NitrosDataType::kInt32;
    case nvidia::gxf::PrimitiveType::kUnsigned32:
      return NitrosDataType::kUnsigned32;
    case nvidia::gxf::PrimitiveType::kInt64:
      return NitrosDataType::kInt64;
    case nvidia::gxf::PrimitiveType::kUnsigned64:
      return NitrosDataType::kUnsigned64;
    case nvidia::gxf::PrimitiveType::kFloat32:
      return NitrosDataType::kFloat32;
    case nvidia::gxf::PrimitiveType::kFloat64:
      return NitrosDataType::kFloat64;
    default:
      return NitrosDataType::kUnknown;
  }
}

nvidia::gxf::Expected<nvidia::gxf::Tensor::stride_array_t> MakeStridesForNitrosTensor(
  const NitrosTensor & tensor,
  const nvidia::gxf::Shape & gxf_shape)
{
  const uint32_t rank = gxf_shape.rank();
  const uint32_t bpe = static_cast<uint32_t>(tensor.bytes_per_element());
  nvidia::gxf::Tensor::stride_array_t strides{};
  if (rank == 0) {
    return strides;
  }
  const auto & nitros_strides = tensor.strides();
  if (nitros_strides.size() == rank) {
    for (uint32_t i = 0; i < rank; ++i) {
      strides[i] = nitros_strides[i];
    }
    return strides;
  }
  return nvidia::gxf::ComputeTrivialStrides(gxf_shape, bpe);
}

}  // namespace

NitrosTensorList NitrosGxfCompatTraits<NitrosTensorList>::CreateFromGxfEntity(
  gxf_context_t context,
  int64_t eid)
{
  auto entity = nvidia::gxf::Entity::Shared(context, eid);
  if (!entity) {
    throw std::runtime_error("NitrosTensorList::CreateFromGxfEntity: failed to open entity");
  }

  auto maybe_tensors = entity->findAll<nvidia::gxf::Tensor>();
  if (!maybe_tensors) {
    throw std::runtime_error(
      "NitrosTensorList::CreateFromGxfEntity: findAll<Tensor> failed: " +
      std::string(GxfResultStr(maybe_tensors.error())));
  }
  const auto & tensor_handles = maybe_tensors.value();
  if (tensor_handles.empty()) {
    throw std::runtime_error("NitrosTensorList::CreateFromGxfEntity: no tensors on entity");
  }

  uint32_t timestamp_sec = 0;
  uint32_t timestamp_nsec = 0;
  auto ts_named = entity->get<nvidia::gxf::Timestamp>("timestamp");
  auto ts_any = ts_named ? ts_named : entity->get<nvidia::gxf::Timestamp>();
  if (ts_any) {
    const uint64_t acqtime = ts_any.value()->acqtime;
    timestamp_sec = static_cast<uint32_t>(acqtime / 1000000000UL);
    timestamp_nsec = static_cast<uint32_t>(acqtime % 1000000000UL);
  }

  gxf_result_t inc = GxfEntityRefCountInc(context, eid);
  if (inc != GXF_SUCCESS) {
    throw std::runtime_error(
      std::string("NitrosTensorList::CreateFromGxfEntity: GxfEntityRefCountInc failed: ") +
      GxfResultStr(inc));
  }

  std::shared_ptr<void> entity_lifetime(nullptr, [context, eid](void *) {
      GxfEntityRefCountDec(context, eid);
    });

  cudaStream_t stream = GetTypeAdapterNitrosContext().getCudaStreamFromNitrosGraph();

  NitrosTensorList list;
  list.set_timestamp_sec(timestamp_sec);
  list.set_timestamp_nsec(timestamp_nsec);
  list.set_storage_type(cudaMemoryTypeDevice);

  for (const auto & th : tensor_handles) {
    auto gxf_tensor = th->get();
    if (!gxf_tensor) {
      throw std::runtime_error("NitrosTensorList::CreateFromGxfEntity: null tensor handle");
    }
    const auto * t = gxf_tensor;
    NitrosDataType dtype = GxfPrimitiveToNitrosDataType(t->element_type());
    if (dtype == NitrosDataType::kUnknown) {
      throw std::runtime_error("NitrosTensorList::CreateFromGxfEntity: unsupported element type");
    }
    NitrosTensorShape shape(t->shape());
    NitrosTensor nitros_tensor;
    const char * cname = th->name();
    const std::string tname = (cname != nullptr && cname[0] != '\0') ? std::string(cname) : "";

    void * dev_ptr = t->pointer();
    const size_t bytes = t->size();
    if (dev_ptr == nullptr || bytes == 0) {
      throw std::runtime_error("NitrosTensorList::CreateFromGxfEntity: empty tensor data");
    }

    (void)nitros_tensor.from_external(
      tname, dev_ptr, bytes, shape, dtype, stream,
      [entity_lifetime](uint8_t * p) {
        (void)p;
        (void)entity_lifetime;
      });
    nitros_tensor.set_name(tname);
    list.add_tensor(std::move(nitros_tensor));
  }

  return list;
}

int64_t NitrosGxfCompatTraits<NitrosTensorList>::CreateGxfEntity(
  gxf_context_t context,
  const NitrosTensorList & msg)
{
  if (msg.handle >= 0) {
    GxfEntityRefCountInc(context, static_cast<gxf_uid_t>(msg.handle));
    return msg.handle;
  }

  auto entity = nvidia::gxf::Entity::New(context);
  if (!entity) {
    throw std::runtime_error(
      "NitrosTensorList::CreateGxfEntity: Entity::New failed: " +
      std::string(GxfResultStr(entity.error())));
  }

  cudaStream_t stream = GetTypeAdapterNitrosContext().getCudaStreamFromNitrosGraph();

  const auto & tensors = msg.get_tensors();
  for (size_t ti = 0; ti < tensors.size(); ++ti) {
    const NitrosTensor & tensor = tensors[ti];
    auto buffer = NitrosBufferAccessor<NitrosTensor>::get_buffer(tensor);
    if (!buffer) {
      throw std::runtime_error("NitrosTensorList::CreateGxfEntity: tensor has no buffer");
    }
    if (tensor.data_type() == NitrosDataType::kUnknown) {
      throw std::runtime_error("NitrosTensorList::CreateGxfEntity: unknown NitrosDataType");
    }

    std::string comp_name = tensor.get_name();
    if (comp_name.empty()) {
      comp_name = "tensor_" + std::to_string(ti);
    }

    auto gxf_tensor = entity->add<nvidia::gxf::Tensor>(comp_name.c_str());
    if (!gxf_tensor) {
      throw std::runtime_error(
        "NitrosTensorList::CreateGxfEntity: add<Tensor> failed: " +
        std::string(GxfResultStr(gxf_tensor.error())));
    }

    std::vector<int32_t> dims = tensor.shape().dims();
    nvidia::gxf::Shape gxf_shape(dims);
    const uint64_t bpe = tensor.bytes_per_element();
    const nvidia::gxf::PrimitiveType prim = GetPrimitiveType(tensor.data_type());

    const uint8_t * raw_ptr = buffer->get_data();
    void * ptr = const_cast<uint8_t *>(raw_ptr);
    if (ptr == nullptr) {
      throw std::runtime_error("NitrosTensorList::CreateGxfEntity: get_data() returned null");
    }
    if (tensor.tensor_size() > buffer->size()) {
      throw std::runtime_error("NitrosTensorList::CreateGxfEntity: tensor larger than buffer");
    }

    nvidia::gxf::Expected<nvidia::gxf::Tensor::stride_array_t> strides =
      MakeStridesForNitrosTensor(tensor, gxf_shape);

    auto buffer_keep = buffer;
    auto wrap_result = gxf_tensor.value()->wrapMemory(
      gxf_shape,
      prim,
      bpe,
      strides,
      nvidia::gxf::MemoryStorageType::kDevice,
      ptr,
      [buffer_keep](void * p) -> nvidia::gxf::Expected<void> {
        (void)p;
        (void)buffer_keep;
        return nvidia::gxf::Success;
      });

    if (!wrap_result) {
      throw std::runtime_error(
        "NitrosTensorList::CreateGxfEntity: wrapMemory failed: " +
        std::string(GxfResultStr(wrap_result.error())));
    }
    (void)stream;
  }

  const uint64_t acqtime =
    static_cast<uint64_t>(msg.get_timestamp_sec()) * 1000000000ULL +
    static_cast<uint64_t>(msg.get_timestamp_nsec());
  auto ts = entity->add<nvidia::gxf::Timestamp>("timestamp");
  if (!ts) {
    throw std::runtime_error(
      "NitrosTensorList::CreateGxfEntity: add<Timestamp>(\"timestamp\") failed: " +
      std::string(GxfResultStr(ts.error())));
  }
  ts.value()->acqtime = acqtime;

  const int64_t eid = entity->eid();
  GxfEntityRefCountInc(context, eid);
  return eid;
}

NITROS_GXF_COMPAT_FORMATS_BEGIN(NitrosTensorList)
NITROS_GXF_COMPAT_FORMAT_ADD(nitros_tensor_list_nchw_t)
NITROS_GXF_COMPAT_FORMAT_ADD(nitros_tensor_list_nhwc_t)
NITROS_GXF_COMPAT_FORMAT_ADD(nitros_tensor_list_nchw_rgb_f32_t)
NITROS_GXF_COMPAT_FORMAT_ADD(nitros_tensor_list_nhwc_rgb_f32_t)
NITROS_GXF_COMPAT_FORMAT_ADD(nitros_tensor_list_nchw_bgr_f32_t)
NITROS_GXF_COMPAT_FORMAT_ADD(nitros_tensor_list_nhwc_bgr_f32_t)
NITROS_GXF_COMPAT_FORMATS_END()

NITROS_GXF_COMPAT_REGISTER_CONVERTER(NitrosTensorList)

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // NITROS_GXF_COMPAT_MODE
