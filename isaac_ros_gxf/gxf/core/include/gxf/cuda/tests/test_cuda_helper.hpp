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
#ifndef NVIDIA_GXF_CUDA_TESTS_TEST_CUDA_HELPER_HPP
#define NVIDIA_GXF_CUDA_TESTS_TEST_CUDA_HELPER_HPP

#include <cublas_v2.h>
#include <algorithm>
#include <limits>
#include <numeric>
#include <queue>
#include <random>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "convolution.h"

#include "common/assert.hpp"
#include "gxf/cuda/cuda_buffer.hpp"
#include "gxf/cuda/cuda_common.hpp"
#include "gxf/cuda/cuda_event.hpp"
#include "gxf/cuda/cuda_stream.hpp"
#include "gxf/cuda/cuda_stream_id.hpp"
#include "gxf/cuda/cuda_stream_pool.hpp"
#include "gxf/std/allocator.hpp"
#include "gxf/std/codelet.hpp"
#include "gxf/std/receiver.hpp"
#include "gxf/std/tensor.hpp"
#include "gxf/std/transmitter.hpp"

#define CHECK_CUBLUS_ERROR(cu_result, fmt, ...)                         \
  do {                                                                  \
    cublasStatus_t err = (cu_result);                                   \
    if (err != CUBLAS_STATUS_SUCCESS) {                                 \
      GXF_LOG_ERROR(fmt ", cublas_error: %d", ##__VA_ARGS__, (int)err); \
      return Unexpected{GXF_FAILURE};                                   \
    }                                                                   \
  } while (0)

namespace nvidia {
namespace gxf {
namespace test {
namespace cuda {

constexpr static const char* kStreamName0 = "CudaStream0";
constexpr static int kDefaultDevId = 0;
static const Shape kInitTensorShape{1024, 2048};

// The base class with cuda stream utils
class StreamBasedOps : public Codelet {
 public:
  static Expected<Handle<CudaStream>> getStream(Entity& message) {
    auto stream_id = message.get<CudaStreamId>();
    GXF_ASSERT(stream_id, "failed to find cudastreamid");
    auto stream =
        Handle<CudaStream>::Create(stream_id.value().context(), stream_id.value()->stream_cid);
    GXF_ASSERT(stream, "create cudastream from cid failed");
    GXF_ASSERT(stream.value(), "cudastream handle is null");
    return stream;
  }

  static Expected<void> addStream(Entity& message, Handle<CudaStream>& stream,
                                  const char* name = nullptr) {
    auto stream_id = message.add<CudaStreamId>(name);
    GXF_ASSERT(stream_id, "failed to add cudastreamid");
    stream_id.value()->stream_cid = stream.cid();
    GXF_ASSERT(stream_id.value()->stream_cid != kNullUid, "stream_cid is null");
    return Success;
  }

  static Expected<Handle<Tensor>> addTensor(Entity& message, Handle<Allocator> pool,
                                            const TensorDescription& description) {
    GXF_ASSERT(pool, "pool is not set");
    auto tensor = message.add<Tensor>(description.name.c_str());
    GXF_ASSERT(tensor, "failed to add message tensor");
    const uint64_t bytes_per_element = description.element_type == PrimitiveType::kCustom
                                           ? description.bytes_per_element
                                           : PrimitiveTypeSize(description.element_type);

    auto result = tensor.value()->reshapeCustom(description.shape, description.element_type,
                                                bytes_per_element, description.strides,
                                                description.storage_type, pool);
    GXF_ASSERT(result, "reshape tensor:%s failed", description.name.c_str());
    return tensor;
  }

  Expected<Handle<CudaEvent>> addNewEvent(Entity& message, const char* name = nullptr) const {
    GXF_ASSERT(opsEvent(), "cuda stream ops event is not initialized");

    auto mabe_event = message.add<CudaEvent>(name);
    GXF_ASSERT(mabe_event, "failed to add cudaevent");
    auto& event = mabe_event.value();
    auto ret = event->initWithEvent(ops_event_->event().value(), ops_event_->dev_id());
    GXF_ASSERT(ret, "failed to init with event into message");
    GXF_ASSERT(event->event(), "stream_cid is null");
    return mabe_event;
  }

  Expected<void> initOpsEvent() {
    if (ops_event_) { return Success; }
    auto ops_event_entity = Entity::New(context());
    GXF_ASSERT(ops_event_entity, "New event entity failed");
    ops_event_holder_ = ops_event_entity.value();
    auto maybe_event = ops_event_holder_.add<CudaEvent>("ops_event");
    GXF_ASSERT(maybe_event, "failed to init ops cudaevent");
    ops_event_ = maybe_event.value();
    auto ret = ops_event_->init(0, kDefaultDevId);
    if (!ret) {
      GXF_LOG_ERROR("failed to init Ops event");
      return ForwardError(ret);
    }
    return ret;
  }

  const Handle<CudaEvent>& opsEvent() const { return ops_event_; }

 private:
  Handle<CudaEvent> ops_event_;
  Entity ops_event_holder_;
};

// Generate cuda tensor map with cudastreams for transmitter cuda_tx,
// Generate host tensor map for transmitter host_tx
class StreamTensorGenerator : public StreamBasedOps {
 public:
  gxf_result_t initialize() override {
    GXF_ASSERT(stream_pool_.get(), "stream pool is not set");
    auto stream = stream_pool_->allocateStream();
    GXF_ASSERT(stream, "allocating stream failed");
    stream_ = std::move(stream.value());
    GXF_ASSERT(stream_->stream(), "allocated stream is not initialized.");
    return GXF_SUCCESS;
  }

  gxf_result_t deinitialize() override {
    auto result = stream_pool_->releaseStream(stream_);
    return ToResultCode(result);
  }

  gxf_result_t tick() override {
    Expected<Entity> maybe_dev_msg = Entity::New(context());
    GXF_ASSERT(maybe_dev_msg, "New dev message failed");
    auto& dev_msg = maybe_dev_msg.value();

    Expected<Entity> maybe_host_msg = Entity::New(context());
    GXF_ASSERT(maybe_host_msg, "New host message failed");
    auto& host_msg = maybe_host_msg.value();

    auto ret = createTensors(dev_msg, host_msg);
    GXF_ASSERT(ret, "creating tensors failed");
    GXF_ASSERT(stream_, "stream is not allocated");

    ret = addStream(dev_msg, stream_, kStreamName0);
    GXF_ASSERT(ret, "stream tensor generator adding stream failed");

    ret = cuda_tx_->publish(dev_msg);
    if (!ret) {
      GXF_LOG_ERROR("stream tensor generator publishing cuda tensors failed");
      return ToResultCode(ret);
    }

    if (host_tx_.get()) {
      ret = host_tx_->publish(host_msg);
      if (!ret) {
        GXF_LOG_ERROR("stream tensor generator publishing cuda tensors failed");
        return ToResultCode(ret);
      }
    }

    return GXF_SUCCESS;
  }

  gxf_result_t registerInterface(Registrar* registrar) override {
    Expected<void> result;
    result &= registrar->parameter(cuda_tx_, "cuda_tx", "transmitter of cuda tensors", "");
    result &= registrar->parameter(host_tx_, "host_tx", "transmitter of host tensors", "",
                                   Handle<Transmitter>());
    result &= registrar->parameter(cuda_tensor_pool_, "cuda_tensor_pool", "Cuda Tensor Pool", "");
    result &= registrar->parameter(host_tensor_pool_, "host_tensor_pool", "Host Tensor Pool", "");
    result &= registrar->parameter(stream_pool_, "stream_pool", "Cuda Stream Pool", "");
    return ToResultCode(result);
  }

 private:
  Expected<void> createTensors(Entity& dev_msg, Entity& host_msg) {
    Shape shape = kInitTensorShape;
    for (size_t i = 0; i < 2; ++i) {
      TensorDescription dev_desc{"cuda_tensor", MemoryStorageType::kDevice, shape,
                                 PrimitiveType::kFloat32};
      auto cuda_tensor_ret = addTensor(dev_msg, cuda_tensor_pool_, dev_desc);
      GXF_ASSERT(cuda_tensor_ret, "Generator dev message adding tensor failed.");
      auto& cuda_tensor = cuda_tensor_ret.value();
      Expected<float*> cuda_data = cuda_tensor->data<float>();
      if (!cuda_data) { return ForwardError(cuda_data); }

      TensorDescription host_desc{"host_tensor", MemoryStorageType::kHost, shape,
                                  PrimitiveType::kFloat32};
      auto host_tensor_ret = addTensor(host_msg, host_tensor_pool_, host_desc);
      GXF_ASSERT(host_tensor_ret, "Generator host message adding tensor failed.");
      auto& host_tensor = host_tensor_ret.value();
      Expected<float*> host_data = host_tensor->data<float>();
      if (!host_data) { return ForwardError(host_data); }

      for (size_t j = 0; j < shape.size(); ++j) {
        host_data.value()[j] = (j % 100) + 1.0f;
      }

      cudaError_t error = cudaMemcpy(cuda_data.value(), host_data.value(), cuda_tensor->size(),
                                     cudaMemcpyHostToDevice);
      CHECK_CUDA_ERROR(error, "StreamTensorGenerator cuda memory cpy H2D failed.");
    }

    return Success;
  }

  Parameter<Handle<Transmitter>> cuda_tx_;
  Parameter<Handle<Transmitter>> host_tx_;
  Parameter<Handle<Allocator>> cuda_tensor_pool_;
  Parameter<Handle<Allocator>> host_tensor_pool_;
  Parameter<Handle<CudaStreamPool>> stream_pool_;

  Handle<CudaStream> stream_;
};

// Dot product execution base class
class DotProductExe {
 private:
  Handle<Receiver> rx_;
  Handle<Transmitter> tx_;
  Handle<Allocator> tensor_pool_;

 public:
  DotProductExe() = default;
  virtual ~DotProductExe() = default;
  void setEnv(const Handle<Receiver>& rx, const Handle<Transmitter>& tx,
              const Handle<Allocator>& pool) {
    rx_ = rx;
    tx_ = tx;
    tensor_pool_ = pool;
  }

  virtual Expected<void> dotproduct_i(float* in0, float* in1, float* out, int32_t row,
                                      int32_t column, Entity& in_msg, Entity& out_msg) = 0;

  Expected<void> execute(const char* out_tensor_name = "") {
    GXF_ASSERT(rx_ && tx_ && tensor_pool_, "dotproduct received empty in_msg");

    Expected<Entity> in_msg = rx_->receive();
    GXF_ASSERT(in_msg, "dotproduct received empty in_msg");

    // get tensors
    auto in_tensors = in_msg->findAll<Tensor>();
    GXF_ASSERT(in_tensors, "failed to find Tensors in in_msg");
    GXF_ASSERT(in_tensors->size() == 2, "doesn't find Tensors in in_msg");
    GXF_ASSERT(in_tensors->at(0).value()->rank() == 2, "Input tensor rank is not 2");
    int32_t column = in_tensors->at(0).value()->shape().dimension(1);
    int32_t row = in_tensors->at(0).value()->shape().dimension(0);
    MemoryStorageType mem_type = in_tensors->at(0).value()->storage_type();
    PrimitiveType data_type = in_tensors->at(0).value()->element_type();
    float* in_data[2] = {nullptr};
    for (size_t i = 0; i < in_tensors->size(); ++i) {
      GXF_ASSERT(in_tensors->at(i).value(), "Input Tensor Handle is empty");
      GXF_ASSERT(in_tensors->at(i).value()->rank() == 2, "Input tensor rank is not 2");
      in_data[i] = ValuePointer<float>(in_tensors->at(i).value()->pointer());
    }

    Expected<Entity> output = Entity::New(in_msg->context());
    GXF_ASSERT(output, "Creating dotproduct output tensor failed.");
    Shape out_shape{row};
    GXF_ASSERT(out_shape.rank() == 1 && out_shape.size() == static_cast<uint64_t>(row),
               "output_shape is not correct");
    TensorDescription outDesc{out_tensor_name, mem_type, out_shape, data_type};
    auto out_tensor = StreamBasedOps::addTensor(output.value(), tensor_pool_, outDesc);
    GXF_ASSERT(out_tensor && out_tensor.value(), "cuda dotproduct output tensor is not found");
    float* out_data = ValuePointer<float>(out_tensor.value()->pointer());

    auto ret =
        dotproduct_i(in_data[0], in_data[1], out_data, row, column, in_msg.value(), output.value());
    GXF_ASSERT(ret, "dotproduct execute with implementation failed");

    ret = tx_->publish(output.value());
    GXF_ASSERT(ret, "dotproduct publishing tensors failed");
    return ret;
  }
};

// Cublas Dot product Operators
class CublasDotProduct : public StreamBasedOps {
 public:
  // Culblas dot production execution class
  class CublasDotProductExe : public DotProductExe {
   private:
    cublasHandle_t handle_ = nullptr;
    CublasDotProduct* codelet_ = nullptr;

   public:
    CublasDotProductExe(CublasDotProduct* codelet) : codelet_(codelet) {}
    ~CublasDotProductExe() {
      if (handle_) { cublasDestroy(handle_); }
    }
    Expected<void> dotproduct_i(float* in0, float* in1, float* out, int32_t row, int32_t column,
                                Entity& in_msg, Entity& out_msg) override {
      // locate stream
      auto maybe_stream = StreamBasedOps::getStream(in_msg);
      GXF_ASSERT(maybe_stream && maybe_stream.value(), "get stream from in_msg failed");
      auto& stream = maybe_stream.value();
      auto ret = StreamBasedOps::addStream(out_msg, stream);
      GXF_ASSERT(ret, "adding cudastream into dotproduct output message failed.");

      int gpu_id = stream->dev_id();
      if (gpu_id >= 0) {
        CHECK_CUDA_ERROR(cudaSetDevice(gpu_id), "failed to set deviceid: %d", gpu_id);
      }

      if (!handle_) {
        CHECK_CUBLUS_ERROR(cublasCreate(&handle_), "failed to create cublas handle");
      }
      auto custream_id = stream->stream();
      GXF_ASSERT(custream_id, "cudastream id is invalid");
      CHECK_CUBLUS_ERROR(cublasSetStream(handle_, custream_id.value()), "cublas set stream failed");

      for (int i = 0; i < row; ++i) {
        CHECK_CUBLUS_ERROR(
            cublasSdot(handle_, column, in0 + column * i, 1, in1 + column * i, 1, out + i),
            "cublasSdot failed on row :%d", i);
      }

      auto maybe_event = codelet_->addNewEvent(out_msg, "cudotproduct_event");
      GXF_ASSERT(maybe_event, "failed to add cublas dot product event");
      // Recording this event here causes a leak
      // ret = stream->record(maybe_event.value(), in_msg,
      //                      []() { GXF_LOG_ERROR("cublas dotproduct event synced"); });
      // GXF_ASSERT(ret, "cublas dotproduct record event failed");
      return ret;
    }
  };

  CublasDotProduct() : exec_(this) {}

  gxf_result_t initialize() override {
    GXF_ASSERT(tensor_pool_.get() && rx_.get() && tx_.get(), "params not set");
    exec_.setEnv(rx_.get(), tx_.get(), tensor_pool_.get());
    return GXF_SUCCESS;
  }

  gxf_result_t start() override { return ToResultCode(initOpsEvent()); }

  gxf_result_t tick() override {
    auto ret = exec_.execute("cublasdotproduct_tensor");
    return ToResultCode(ret);
  }

  gxf_result_t registerInterface(Registrar* registrar) override {
    Expected<void> result;
    result &= registrar->parameter(tx_, "tx", "transmitter of tensors", "");
    result &= registrar->parameter(rx_, "rx", "receiver of tensors", "");
    result &= registrar->parameter(tensor_pool_, "tensor_pool", "Tensor Pool", "");
    return ToResultCode(result);
  }

 private:
  Parameter<Handle<Transmitter>> tx_;
  Parameter<Handle<Receiver>> rx_;
  Parameter<Handle<Allocator>> tensor_pool_;

  CublasDotProductExe exec_;
};

// CPU Dot product Operators
class HostDotProduct : public Codelet {
 public:
  // CPU dot production execution class
  class HostDotProductExe : public DotProductExe {
   public:
    HostDotProductExe() = default;
    ~HostDotProductExe() = default;
    Expected<void> dotproduct_i(float* in0, float* in1, float* out, int32_t row, int32_t column,
                                Entity& in_msg, Entity& out_msg) override {
      for (int i = 0; i < row; ++i) {
        float sum = 0.0;
        float* x = in0 + column * i;
        float* y = in1 + column * i;
        for (int j = 0; j < column; ++j) { sum += x[j] * y[j]; }
        out[i] = sum;
      }
      return Success;
    }
  };

  gxf_result_t initialize() override {
    GXF_ASSERT(tensor_pool_.get() && rx_.get() && tx_.get(), "params not set");
    exec_.setEnv(rx_.get(), tx_.get(), tensor_pool_.get());
    return GXF_SUCCESS;
  }

  gxf_result_t tick() override {
    auto ret = exec_.execute("host_dotproduct_tensor");
    return ToResultCode(ret);
  }

  gxf_result_t registerInterface(Registrar* registrar) override {
    Expected<void> result;
    result &= registrar->parameter(tx_, "tx", "transmitter of tensors", "");
    result &= registrar->parameter(rx_, "rx", "receiver of tensors", "");
    result &= registrar->parameter(tensor_pool_, "tensor_pool", "Tensor Pool", "");
    return ToResultCode(result);
  }

 private:
  Parameter<Handle<Transmitter>> tx_;
  Parameter<Handle<Receiver>> rx_;
  Parameter<Handle<Allocator>> tensor_pool_;

  HostDotProductExe exec_;
};

// Stream based Memory copy from device to host
class MemCpy2Host : public StreamBasedOps {
 public:
  gxf_result_t start() override { return ToResultCode(initOpsEvent()); }

  gxf_result_t tick() override {
    auto in = rx_->receive();
    GXF_ASSERT(in, "rx received empty message");
    auto maybe_tensor = in.value().get<Tensor>();
    GXF_ASSERT(maybe_tensor, "tensor not found");
    auto& in_tensor = maybe_tensor.value();
    byte* in_data = in_tensor->pointer();

    Expected<Entity> out_msg = Entity::New(context());
    TensorDescription out_desc{in_tensor.name(), MemoryStorageType::kHost, in_tensor->shape(),
                               in_tensor->element_type()};
    auto maybe_out_tensor = addTensor(out_msg.value(), tensor_pool_, out_desc);
    GXF_ASSERT(maybe_out_tensor, "Memcpy host message adding tensor failed.");
    auto& out_tensor = maybe_out_tensor.value();
    byte* out_data = out_tensor->pointer();

    auto maybe_stream = getStream(in.value());
    GXF_ASSERT(maybe_stream && maybe_stream.value(), "get stream from in failed");
    auto& stream = maybe_stream.value();
    auto ret = addStream(out_msg.value(), stream);
    GXF_ASSERT(ret, "adding cudastream into memcpy output message failed.");

    // wrap cuda operations since CHECK_CUDA_ERROR return Expected<void>
    ret = [&, this]() -> Expected<void> {
      int gpu_id = stream->dev_id();
      if (gpu_id >= 0) {
        CHECK_CUDA_ERROR(cudaSetDevice(gpu_id), "failed to set deviceid: %d", gpu_id);
      }
      cudaError_t error = cudaMemcpyAsync(out_data, in_data, in_tensor->size(),
                                          cudaMemcpyDeviceToHost, stream->stream().value());
      CHECK_CUDA_ERROR(error, "CUDA memory cpy to host failed.");
      return Success;
    }();
    GXF_ASSERT(ret, "CUDA memory cpy to host failed.");

    auto maybe_event = addNewEvent(out_msg.value(), "memcpy_event");
    GXF_ASSERT(maybe_event, "failed to add memcpy_event");
    // Recording this event here causes a leak
    // ret = stream->record(maybe_event.value(), in.value(),
    //                      []() { GXF_LOG_ERROR("memcpy_to_host event synced"); });
    // GXF_ASSERT(ret, "memcpy_to_host record event failed");

    ret = tx_->publish(out_msg.value());
    GXF_ASSERT(ret, "memcpy_to_host publishing tensors failed");

    return ToResultCode(ret);
  }

  gxf_result_t registerInterface(Registrar* registrar) override {
    Expected<void> result;
    result &= registrar->parameter(tx_, "tx", "transmitter of tensors", "");
    result &= registrar->parameter(rx_, "rx", "receiver of tensors", "");
    result &= registrar->parameter(tensor_pool_, "tensor_pool", "Tensor output pool", "");
    return ToResultCode(result);
  }

 private:
  Parameter<Handle<Transmitter>> tx_;
  Parameter<Handle<Receiver>> rx_;
  Parameter<Handle<Allocator>> tensor_pool_;
};

// Equal verification
class VerifyEqual : public Codelet {
 public:
  gxf_result_t tick() override {
    GXF_LOG_DEBUG("verifying frame: %d", count_++);
    auto in0 = rx0_->receive();
    GXF_ASSERT(in0, "rx0 received empty message");
    auto in1 = rx1_->receive();
    GXF_ASSERT(in1, "rx1 received empty message");

    // get tensors
    auto maybe_tensor0 = in0.value().get<Tensor>();
    GXF_ASSERT(maybe_tensor0, "tensor0 not found");
    auto maybe_tensor1 = in1.value().get<Tensor>();
    GXF_ASSERT(maybe_tensor1, "tensor1 not found");
    auto& tensor0 = maybe_tensor0.value();
    auto& tensor1 = maybe_tensor1.value();
    GXF_ASSERT(std::string(tensor0.name()) != std::string(tensor1.name()),
               "2 tensor name should not same");
    GXF_ASSERT(tensor0->shape() == tensor1->shape(), "2 tensors' shape not matched");
    GXF_ASSERT(tensor0->element_type() == tensor1->element_type(),
               "2 tensor's element type not matched");
    GXF_ASSERT(tensor0->storage_type() == tensor1->storage_type(),
               "2 tensor's storage_type not matched");
    GXF_ASSERT(tensor0->storage_type() == MemoryStorageType::kHost,
               "very tensor storage_type is not from host");

    float* data0 = tensor0->data<float>().value();
    float* data1 = tensor1->data<float>().value();
    uint64_t count = tensor0->element_count();
    for (uint64_t i = 0; i < count; ++i) {
      // printf("i:%d, [%f], [%f]\n", static_cast<int>(i), data0[i], data1[i]);
      GXF_ASSERT(fequal(data0[i], data1[i]), "data0[%d]: %f but data1: %f.", static_cast<int>(i),
                 data0[i], data1[i]);
    }

    return GXF_SUCCESS;
  }

  gxf_result_t registerInterface(Registrar* registrar) override {
    Expected<void> result;
    result &= registrar->parameter(rx0_, "rx0", "receiver0 of tensors", "");
    result &= registrar->parameter(rx1_, "rx1", "receiver1 of tensors", "");
    return ToResultCode(result);
  }

 private:
  bool fequal(float a, float b) {
    if (fabs(a - b) <= std::numeric_limits<float>::epsilon()) return true;
    return false;
  }
  Parameter<Handle<Receiver>> rx0_;
  Parameter<Handle<Receiver>> rx1_;
  int32_t count_ = 0;
};

// Creates a CudaBuffer object on each iteration with memory allocated in an
// asynchronous manner. Downstream receivers can either enqueue more work on to
// CudaStream associated with the CudaBuffer or sync on the stream before accessing the memory
class CudaAsyncBufferGenerator : public Codelet {
 public:
  virtual ~CudaAsyncBufferGenerator() = default;

  gxf_result_t registerInterface(Registrar* registrar) override {
    Expected<void> result;
    result &= registrar->parameter(signal_, "signal", "Signal",
              "Transmitter channel publishing messages to other graph entities");
    result &= registrar->parameter(allocator_, "allocator", "Allocator",
              "A cuda allocator component that can serve device memory asynchronously");
    result &= registrar->parameter(stream_pool_, "stream_pool", "StreamPool",
              "A cuda stream pool component");
    result &= registrar->parameter(size_, "size", "Size",
              "Size of the CudaBuffer to be allocated", 1UL);
    return ToResultCode(result);
  }

  gxf_result_t tick() override {
    auto maybe_message = Entity::New(context());
    if (!maybe_message) {
      GXF_LOG_ERROR("Failure creating message entity.");
      return maybe_message.error();
    }

    auto message = maybe_message.value();
    auto maybe_buffer = message.add<CudaBuffer>(kDefaultCudaBufferName);
    if (!maybe_buffer) {
      GXF_LOG_ERROR("Failure creating cuda buffer");
      return maybe_buffer.error();
    }

    auto buffer = maybe_buffer.value();
    auto code = buffer->resizeAsync(allocator_, stream_pool_, size_);

    auto min = std::numeric_limits<float>::min();
    auto max = std::numeric_limits<float>::max();
    std::uniform_real_distribution<float> distribution(min, max);
    std::vector<float> elements;
    auto element_count = size_ / sizeof(float);

    for (size_t idx = 0; idx < element_count; idx++) {
      elements.push_back(distribution(generator_));
    }

    auto stream_component = buffer->stream();
    auto stream = stream_component->stream().value();

    const cudaError_t error = cudaMemcpyAsync(buffer->pointer(), elements.data(),
                                              size_, cudaMemcpyHostToDevice, stream);
    CHECK_CUDA_ERROR_RESULT(error, "cudaMemcpyAsync failed");

    auto result = signal_->publish(message);
    GXF_LOG_INFO("Message Sent");
    return GXF_SUCCESS;
  }

 private:
  Parameter<Handle<Transmitter>> signal_;
  Parameter<Handle<CudaAllocator>> allocator_;
  Parameter<Handle<CudaStreamPool>> stream_pool_;
  Parameter<size_t> size_;
  std::default_random_engine generator_;
};

// Performs an async 2D convolution operation on
// incoming CudaBuffer objects.
class Convolve2D: public Codelet {
 public:
  virtual ~Convolve2D() = default;

  gxf_result_t registerInterface(Registrar* registrar) override {
    Expected<void> result;
    result &= registrar->parameter(kernel_, "kernel", "Kernel",
              "2d matrix of float values represented as a vector of vectors");
    result &= registrar->parameter(output_, "output", "Output",
              "Transmitter channel publishing messages to other graph entities");
    result &= registrar->parameter(input_, "input", "Input",
              "Receiver channel receiving messages from other graph entities");
    result &= registrar->parameter(allocator_, "allocator", "Allocator",
              "A cuda allocator component that can serve device memory asynchronously");
    return ToResultCode(result);
  }

  gxf_result_t start() override {
    auto& kernel = kernel_.get();
    size_t kernel_size = kernel.size();

    size_t index = 0;
    float kernel_flat_host[kernel_size * kernel_size]; // NOLINT
    for (const auto& k : kernel) {
      GXF_ASSERT_EQ(k.size(), kernel_size);
      for (const auto& value : k) {
        kernel_flat_host[index] = value;
        ++index;
      }
    }

    auto maybe_kernel = allocator_->allocate(kernel_size * kernel_size * sizeof(float),
                                             MemoryStorageType::kDevice);
    if (!maybe_kernel) {
      return maybe_kernel.error();
    }

    kernel_flat_ = maybe_kernel.value();
    const cudaError_t error =
        cudaMemcpy(kernel_flat_, kernel_flat_host, kernel_size * kernel_size * sizeof(float),
                   cudaMemcpyHostToDevice);
    if (error != cudaSuccess) {
        GXF_LOG_ERROR("CUDA Mem copy Error: %s\n", cudaGetErrorString(error));
        return GXF_FAILURE;
    }
    return GXF_SUCCESS;
  }


  gxf_result_t stop() override {
    entity_queue_map_.clear();
    return ToResultCode(allocator_->free(kernel_flat_));
  }

  gxf_result_t tick() override {
    return ToResultCode(_tick());
  }

  // payload struct to access on the callback used to
  // dereference the underlying message entity
  typedef struct callBackData {
    Entity data;
    Convolve2D* ptr;
  } callBackData;

  static void CUDART_CB cudaEntityFreeCallback(void* data_ptr) {
    auto data = reinterpret_cast<callBackData*>(data_ptr);
    auto convolve = data->ptr;
    GXF_LOG_VERBOSE("Received entity free callback from cuda stream for "
    "cuda buffer in message entity E[%05ld]", data->data.eid());
    // Entities consumed until now cannot be freed just yet in the host callback function.
    // Host callback function gets executed from a cuda driver thread which cannot be used
    // to call another cuda runtime api like cudaFreeAsync. Hence keep track of the
    // entities to be freed the next time this codelet is executed.
    std::unique_lock<std::shared_mutex> lock(convolve->mutex_);
    convolve->entity_free_list_.push(data->data.eid());
  }

  Expected<void> _tick() {
    // Dequeue any previously consumed message entities
    {
      std::unique_lock<std::shared_mutex> lock(mutex_);
      while (!entity_free_list_.empty()) {
        auto element = entity_free_list_.front();
        entity_queue_map_.erase(element);
        entity_free_list_.pop();
      }
    }

    auto in_message = GXF_UNWRAP_OR_RETURN(input_->receive());
    GXF_LOG_INFO("Message Received: %d", this->count_);
    this->count_ = this->count_ + 1;
    if (in_message.is_null()) {
      return Unexpected{GXF_CONTRACT_MESSAGE_NOT_AVAILABLE};
    }

    auto in_buffer = GXF_UNWRAP_OR_RETURN(in_message.get<CudaBuffer>());
    auto out_message = GXF_UNWRAP_OR_RETURN(Entity::New(context()));
    auto out_buffer = GXF_UNWRAP_OR_RETURN(out_message.add<CudaBuffer>(kDefaultCudaBufferName));

    auto stream_pool = in_buffer->streamPool();
    auto stream_handle = in_buffer->transferStreamOwnership();
    auto code = out_buffer->resizeAsync(allocator_, stream_pool, in_buffer->size(), stream_handle);

    const int width = 32, height = 32;
    auto cu_stream = GXF_UNWRAP_OR_RETURN(stream_handle->stream());

    convolveKernel(reinterpret_cast<float*>(in_buffer->pointer()),
                   reinterpret_cast<float*>(kernel_flat_),
                   reinterpret_cast<float*>(out_buffer->pointer()),
                   width, height, kernel_.get().size(), cu_stream);

    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess) {
        GXF_LOG_ERROR("CUDA convolveKernel Error: %s\n", cudaGetErrorString(err));
        return Unexpected{GXF_FAILURE};
    }

    // store the incoming message entity to prevent the cuda buffer getting released
    // before it is consumed. The in_message entity is only released once the kernel
    // has finished its execution
    auto eid = in_message.eid();
    {
      std::unique_lock<std::shared_mutex> lock(mutex_);
      entity_queue_map_.insert({eid, callBackData{std::move(in_message), this}});
    }
    auto it = entity_queue_map_.find(eid);


    GXF_RETURN_IF_ERROR(in_buffer->registerCallbackOnStream(cudaEntityFreeCallback,
                                                    reinterpret_cast<void*>(&it->second)));

    auto stream_id = GXF_UNWRAP_OR_RETURN(out_message.add<CudaStreamId>());
    stream_id->stream_cid = stream_handle.cid();
    return output_->publish(out_message);
  }

 private:
  Parameter<Handle<CudaAllocator>> allocator_;
  Parameter<std::vector<std::vector<float>>> kernel_;
  Parameter<Handle<Transmitter>> output_;
  Parameter<Handle<Receiver>> input_;
  byte* kernel_flat_;
  std::unordered_map<gxf_uid_t, callBackData> entity_queue_map_;
  std::queue<gxf_uid_t> entity_free_list_;
  std::shared_mutex mutex_;
  int count_ = 1;
};

// Dequeue a message entity with a cuda buffer and ensures
// the data is available
class CudaBufferRx: public Codelet {
 public:
  virtual ~CudaBufferRx() = default;

  gxf_result_t registerInterface(Registrar* registrar) override {
    Expected<void> result;
    result &= registrar->parameter(signal_, "signal", "Signal",
                        "Channel to receive messages from another graph entity");
    return ToResultCode(result);
  }

  gxf_result_t tick() override {
    auto maybe_message = signal_->receive();
    GXF_LOG_INFO("Message Received: %d", this->count);
    this->count = this->count + 1;
    if (!maybe_message || maybe_message.value().is_null()) {
      return GXF_CONTRACT_MESSAGE_NOT_AVAILABLE;
    }

    auto message = maybe_message.value();
    auto maybe_cuda_buffer = message.get<CudaBuffer>();
    if (!maybe_cuda_buffer) { return ToResultCode(maybe_cuda_buffer); }

    auto cuda_buffer = maybe_cuda_buffer.value();
    auto state = cuda_buffer->state();
    if (state != CudaBuffer::State::DATA_AVAILABLE) {
      GXF_LOG_ERROR("CudaBuffer found in an invalid state %d", static_cast<int>(state));
      return GXF_FAILURE;
    }

    return GXF_SUCCESS;
  }

 private:
  Parameter<Handle<Receiver>> signal_;
  int count = 1;
};

}  // namespace cuda

}  // namespace test

}  // namespace gxf

}  // namespace nvidia

#endif  // NVIDIA_GXF_CUDA_TESTS_TEST_CUDA_HELPER_HPP
