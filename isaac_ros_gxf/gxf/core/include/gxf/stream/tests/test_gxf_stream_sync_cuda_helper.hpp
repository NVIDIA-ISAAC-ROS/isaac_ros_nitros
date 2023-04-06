/*
 * SPDX-FileCopyrightText: Copyright (c) 2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#ifndef NVIDIA_GXF_STREAM_TESTS_TEST_STREAM_CUDA_HELPER_HPP
#define NVIDIA_GXF_STREAM_TESTS_TEST_STREAM_CUDA_HELPER_HPP

#include <cublas_v2.h>
#include <limits>
#include <numeric>
#include <string>
#include <utility>

#include "common/assert.hpp"
#include "gxf/cuda/cuda_common.hpp"
#include "gxf/cuda/cuda_stream_id.hpp"
#include "gxf/cuda/cuda_stream_pool.hpp"
#include "gxf/std/allocator.hpp"
#include "gxf/std/codelet.hpp"
#include "gxf/std/receiver.hpp"
#include "gxf/std/tensor.hpp"
#include "gxf/std/transmitter.hpp"
#include "gxf/stream/stream_nvsci.hpp"
#include "gxf/stream/stream_nvscisync.hpp"

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
namespace stream {
namespace test {

constexpr static const char* kStreamName0 = "CudaStream0";
constexpr static int kDefaultDevId = 0;
static const Shape kInitTensorShape{1024, 2048};

// The base class with cuda stream utils
class StreamBasedOpsNew : public Codelet {
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
  static Expected<void> addStreamSync(StreamSync* stream_sync, SyncType signaler,
                                      SyncType waiter, cudaStream_t stream) {
    GXF_ASSERT_SUCCESS(stream_sync->initialize());

    void *syncObj{nullptr};
    GXF_ASSERT_SUCCESS(stream_sync->allocate_sync_object(signaler, waiter,
                                                         reinterpret_cast<void**>(&syncObj)));
    GXF_ASSERT(syncObj, "syncObj is null");

    GXF_ASSERT_SUCCESS(stream_sync->setCudaStream(signaler, stream));

    GXF_ASSERT_SUCCESS(stream_sync->setCudaStream(waiter, stream));

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
};

// Generate cuda tensor map with cudastreams for transmitter cuda_tx,
// Generate host tensor map for transmitter host_tx
class StreamTensorGeneratorNew : public StreamBasedOpsNew {
 public:
  gxf_result_t initialize() override {
    GXF_ASSERT(stream_pool_.get(), "stream pool is not set");
    auto stream = stream_pool_->allocateStream();
    GXF_ASSERT(stream, "allocating stream failed");
    stream_ = std::move(stream.value());
    GXF_ASSERT(stream_->stream(), "allocated stream is not initialized.");
    return GXF_SUCCESS;
  }
  gxf_result_t start() override { return GXF_SUCCESS; }

  gxf_result_t tick() override {
    Expected<Entity> maybe_dev_msg = Entity::New(context());
    GXF_ASSERT(maybe_dev_msg, "New dev message failed");
    auto& dev_msg = maybe_dev_msg.value();

    Expected<Entity> maybe_host_msg = Entity::New(context());
    GXF_ASSERT(maybe_host_msg, "New host message failed");
    auto& host_msg = maybe_host_msg.value();

    auto ret = addStream(dev_msg, stream_, kStreamName0);
    GXF_ASSERT(ret, "stream tensor generator adding stream failed");

    auto sync_ret = dev_msg.add(stream_sync_.get().tid(), "nvidia::gxf::StreamSync");
    GXF_ASSERT_TRUE(sync_ret.has_value());
    auto maybe_stream_sync = dev_msg.findAll<StreamSync>();
    GXF_ASSERT_TRUE(maybe_stream_sync.has_value());

    auto stream_sync = maybe_stream_sync->at(0).value();
    ret = addStreamSync(stream_sync, static_cast<SyncType>(signaler_.get()),
                        static_cast<SyncType>(waiter_.get()), stream_->stream().value());
    GXF_ASSERT(ret, "Failed to add stream sync");

    ret = createTensors(dev_msg, host_msg, stream_->stream().value());
    GXF_ASSERT(ret, "creating tensors failed");
    GXF_ASSERT(stream_, "stream is not allocated");

    stream_sync->signalSemaphore();

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

  gxf_result_t stop() override { return GXF_SUCCESS; }

  gxf_result_t registerInterface(Registrar* registrar) override {
    Expected<void> result;
    result &= registrar->parameter(cuda_tx_, "cuda_tx", "transmitter of cuda tensors", "");
    result &= registrar->parameter(host_tx_, "host_tx", "transmitter of host tensors", "",
                                   Handle<Transmitter>());
    result &= registrar->parameter(cuda_tensor_pool_, "cuda_tensor_pool", "Cuda Tensor Pool", "");
    result &= registrar->parameter(host_tensor_pool_, "host_tensor_pool", "Host Tensor Pool", "");
    result &= registrar->parameter(stream_pool_, "stream_pool", "Cuda Stream Pool", "");
    result &= registrar->parameter(stream_sync_, "stream_sync",
                                   "Synchronization object across CUDA codelets", "");
    result &= registrar->parameter(signaler_, "signaler", "Signaler type",
                                   "Defines the signaler type. Cuda Signaler (2)",
                                   static_cast<int32_t>(SyncType::GXF_STREAM_SIGNALER_CUDA));
    result &= registrar->parameter(waiter_, "waiter", "Waiter type",
                                   "Defines the waiter type. Cuda Waiter (6)",
                                   static_cast<int32_t>(SyncType::GXF_STREAM_WAITER_CUDA));
    return ToResultCode(result);
  }

 private:
  Expected<void> createTensors(Entity& dev_msg, Entity& host_msg, cudaStream_t stream) {
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

      cudaError_t error = cudaMemcpyAsync(cuda_data.value(), host_data.value(), cuda_tensor->size(),
                                          cudaMemcpyHostToDevice, stream);
      CHECK_CUDA_ERROR(error, "StreamTensorGeneratorNew cuda memory cpy H2D failed.");
    }

    return Success;
  }

  Parameter<Handle<Transmitter>> cuda_tx_;
  Parameter<Handle<Transmitter>> host_tx_;
  Parameter<Handle<Allocator>> cuda_tensor_pool_;
  Parameter<Handle<Allocator>> host_tensor_pool_;
  Parameter<Handle<CudaStreamPool>> stream_pool_;
  Parameter<Handle<StreamSync>> stream_sync_;
  Parameter<int32_t> signaler_;
  Parameter<int32_t> waiter_;

  Handle<CudaStream> stream_;
};

// Dot product execution base class
class DotProductExeNew {
 private:
  Handle<Receiver> rx_;
  Handle<Transmitter> tx_;
  Handle<Allocator> tensor_pool_;

 public:
  DotProductExeNew() = default;
  virtual ~DotProductExeNew() = default;
  void setEnv(const Handle<Receiver>& rx, const Handle<Transmitter>& tx,
              const Handle<Allocator>& pool) {
    rx_ = rx;
    tx_ = tx;
    tensor_pool_ = pool;
  }

  virtual Expected<void> dotproduct_i(float* in0, float* in1, float* out, int32_t row,
                                      int32_t column, Entity& in_msg, Entity& out_msg) = 0;

  Expected<void> executeNew(Entity in_msg, const char* out_tensor_name = "") {
    GXF_ASSERT(rx_ && tx_ && tensor_pool_, "dotproduct received empty in_msg");

    // get tensors
    auto in_tensors = in_msg.findAll<Tensor>();
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

    Expected<Entity> output = Entity::New(in_msg.context());
    GXF_ASSERT(output, "Creating dotproduct output tensor failed.");
    Shape out_shape{row};
    GXF_ASSERT(out_shape.rank() == 1 && out_shape.size() == static_cast<uint64_t>(row),
               "output_shape is not correct");
        TensorDescription outDesc{out_tensor_name, mem_type, out_shape, data_type};
    auto out_tensor = StreamBasedOpsNew::addTensor(output.value(), tensor_pool_, outDesc);
    GXF_ASSERT(out_tensor && out_tensor.value(), "cuda dotproduct output tensor is not found");
    float* out_data = ValuePointer<float>(out_tensor.value()->pointer());
        auto ret =
        dotproduct_i(in_data[0], in_data[1], out_data, row, column, in_msg, output.value());
    GXF_ASSERT(ret, "dotproduct execute with implementation failed");
        ret = tx_->publish(output.value());
    GXF_ASSERT(ret, "dotproduct publishing tensors failed");
        return ret;
  }
};

// Cublas Dot product Operators
class CublasDotProductNew : public StreamBasedOpsNew {
 public:
  // Culblas dot production execution class
  class CublasDotProductExe : public DotProductExeNew {
   private:
    cublasHandle_t handle_ = nullptr;
    CublasDotProductNew* codelet_ = nullptr;

   public:
    CublasDotProductExe(CublasDotProductNew* codelet) : codelet_(codelet) {}
    ~CublasDotProductExe() {
      if (handle_) { cublasDestroy(handle_); }
    }
    Expected<void> dotproduct_i(float* in0, float* in1, float* out, int32_t row, int32_t column,
                                Entity& in_msg, Entity& out_msg) override {
      // locate stream
      auto maybe_stream = StreamBasedOpsNew::getStream(in_msg);
      GXF_ASSERT(maybe_stream && maybe_stream.value(), "get stream from in_msg failed");
      auto& stream = maybe_stream.value();
      auto ret = StreamBasedOpsNew::addStream(out_msg, stream);
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

      return ret;
    }
  };

  CublasDotProductNew() : exec_(this) {}

  gxf_result_t initialize() override {
    GXF_ASSERT(tensor_pool_.get() && rx_.get() && tx_.get(), "params not set");
    exec_.setEnv(rx_.get(), tx_.get(), tensor_pool_.get());
    return GXF_SUCCESS;
  }

  // gxf_result_t start() override { return ToResultCode(initOpsEvent()); }
  gxf_result_t start() override { return GXF_SUCCESS; }
  gxf_result_t stop() override { return GXF_SUCCESS; }

  gxf_result_t tick() override {
    Expected<Entity> in_msg = rx_->receive();

    auto in_stream_sync = in_msg.value().findAll<StreamSync>();
    GXF_ASSERT_TRUE(in_stream_sync.has_value());
    in_stream_sync->at(0).value()->waitSemaphore();

    auto ret = exec_.executeNew(in_msg.value(), "cublasdotproduct_tensor");
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
class HostDotProductNew : public Codelet {
 public:
  // CPU dot production execution class
  class HostDotProductExeNew : public DotProductExeNew {
   public:
    HostDotProductExeNew() = default;
    ~HostDotProductExeNew() = default;
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

  gxf_result_t start() override { return GXF_SUCCESS; }
  gxf_result_t stop() override { return GXF_SUCCESS; }

  gxf_result_t tick() override {
    Expected<Entity> in_msg = rx_->receive();


    auto ret = exec_.executeNew(in_msg.value(), "host_dotproduct_tensor");
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

  HostDotProductExeNew exec_;
};

// Stream based Memory copy from device to host
class MemCpy2HostNew : public StreamBasedOpsNew {
 public:
  gxf_result_t start() override { return GXF_SUCCESS; }
  gxf_result_t stop() override { return GXF_SUCCESS; }

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
class VerifyEqualNew : public Codelet {
 public:
  gxf_result_t start() override { return GXF_SUCCESS; }
  gxf_result_t stop() override { return GXF_SUCCESS; }

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

}  // namespace test

}  // namespace stream

}  // namespace gxf

}  // namespace nvidia

#endif  // NVIDIA_GXF_STREAM_TESTS_TEST_STREAM_CUDA_HELPER_HPP
