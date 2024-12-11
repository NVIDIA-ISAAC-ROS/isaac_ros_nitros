// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#ifndef NVIDIA_GXF_UCX_UCX_ROUTER_HPP_
#define NVIDIA_GXF_UCX_UCX_ROUTER_HPP_

#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <ucp/api/ucp.h>
#include <unistd.h>
#include <list>
#include <memory>
#include <queue>

#include "gxf/std/network_context.hpp"
#include "gxf/std/queue.hpp"

#include "ucx_common.hpp"
#include "ucx_receiver.hpp"
#include "ucx_transmitter.hpp"

#define kMaxRxContexts 1024

namespace nvidia {
namespace gxf {

typedef struct ucx_server_ctx {
    volatile ucp_conn_request_h conn_request;
    ucp_listener_h              listener;
    ucp_worker_h                listener_worker;
    int                         listener_fd;  ///< epoll fd used only when enable_async_ == true
} ucx_server_ctx_t;



typedef struct UcxReceiverContext {
    UcxReceiver             *rx;
    ucx_server_ctx_t        server_context;
    ucp_ep_h                ep;
    ConnState               conn_state;
    ucx_am_data_desc        am_data_desc;
    FixedVector<std::shared_ptr<ucx_am_data_desc>, kMaxRxContexts> headers;
    ucp_worker_h            ucp_worker;
    int                     worker_fd;  ///< epoll fd used only when enable_async_ == true
    int                     index;
} UcxReceiverContext_;

typedef struct UcxTransmitterContext {
    UcxTransmitter          *tx;
    ucp_ep_h                ep;
    ucp_worker_h            ucp_worker;
    bool                    connection_closed;
    int                     index;
} UcxTransmitterContext_;

typedef struct ConnManager {
    int total;
    int connected;
    int closed;
} ConnManager_;


// The class which initializes UCX context and connection worker
class UcxContext : public NetworkContext {
 public:
    gxf_result_t registerInterface(Registrar* registrar) override;
    gxf_result_t initialize() override;
    gxf_result_t deinitialize() override;
    Expected<void> addRoutes(const Entity& entity) override;
    Expected<void> removeRoutes(const Entity& entity) override;

 private:
    gxf_result_t init_context();
    gxf_result_t init_tx(Handle<UcxTransmitter> tx);
    gxf_result_t init_rx(Handle<UcxReceiver> rx);

    /// @brief start server for the case with enable_async_ == false
    void start_server();

    gxf_result_t am_desc_to_iov(std::shared_ptr<UcxReceiverContext> rx_context);
    void destroy_rx_contexts();
    void destroy_tx_contexts();

    // The remaining methods are specific to the case with enable_async_ == true

    /// @brief start server for the case with enable_async_ == true
    void start_server_async_queue();
    /// @brief method run by tx_thread_ when enable_async_ == true
    void poll_queue();
    gxf_result_t wait_for_event();   ///< primary function called by start_server_async_queue
    gxf_result_t epoll_add_worker(std::shared_ptr<UcxReceiverContext> rx_context,
            bool is_listener);
    gxf_result_t progress_work(std::shared_ptr<UcxReceiverContext> rx_context);
    gxf_result_t server_create_ep(std::shared_ptr<UcxReceiverContext> rx_context);
    gxf_result_t init_connection(std::shared_ptr<UcxReceiverContext> rx_context);
    gxf_result_t handle_connections_after_recv();
    gxf_result_t gxf_arm_worker(std::shared_ptr<UcxReceiverContext> rx_context,
            bool is_listener);
    void copy_header_to_am_desc(std::shared_ptr<UcxReceiverContext> rx_context);

    bool close_server_loop_;
    FixedVector<std::shared_ptr<UcxReceiverContext>, kMaxRxContexts> rx_contexts_;
    FixedVector<std::shared_ptr<UcxTransmitterContext>, kMaxRxContexts> tx_contexts_;
    ucp_context_h ucp_context_;
    Parameter<Handle<EntitySerializer>> entity_serializer_;
    Parameter<bool> reconnect_;
    Resource<Handle<GPUDevice>> gpu_device_;
    Parameter<bool> cpu_data_only_;
    Parameter<bool> enable_async_;
    int32_t dev_id_ = 0;

    std::thread t_;  ///< server thread used only when enable_async_ == false

    // The following data members are only used when enable_async_ == true
    ConnManager rx_conns_ = {0};
    std::thread rx_thread_;  ///< async receiver thread used only when enable_async_ == true
    std::thread tx_thread_;  ///< async transmitter thread used only when enable_async_ == true
    std::list<UcxTransmitterSendContext_> pending_send_requests_;
    std::mutex mtx_;
    std::condition_variable cv_;
    bool areTransmittersDone = false;
    int epoll_fd_;
    int efd_signal_;
};

}  // namespace gxf
}  // namespace nvidia

#endif
