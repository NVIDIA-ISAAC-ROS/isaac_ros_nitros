// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef NVIDIA_GXF_UCX_UCX_COMMON_HPP_
#define NVIDIA_GXF_UCX_UCX_COMMON_HPP_

#include <arpa/inet.h> /* inet_addr */

#include <ucp/api/ucp.h>
#include <queue>
#include "gxf/std/allocator.hpp"

#define DEFAULT_UCX_PORT           13337U
#define TEST_AM_ID             0
#define WORKER_PROGRESS_ITERATIONS 5
namespace nvidia {
namespace gxf {

typedef struct _ucx_am_data_desc {
    volatile int    complete;
    void            *desc;
    volatile size_t msg_length;
    volatile size_t header_length;
    void            *recv_buf;
    void            *header;
    volatile size_t num_of_comps;
    bool            receiving_message;
    ucs_memory_type_t mem_type;
} ucx_am_data_desc;
/**
 * Stream request context. Holds a value to indicate whether or not the
 * request is completed.
 */
typedef struct test_req {
    int complete;
    char *header;
} test_req_t;

  typedef enum {
    INIT,
    CONNECTED,
    RESET,
    CLOSED,
    RECEIVE_MESSAGE
  } ConnState;

#define CHECK_RETURN_FAILURE(value) \
    if ((value) != GXF_SUCCESS) return (value);

typedef struct UcxTransmitterSendContext {
    Entity                  entity;
    ucp_worker_h            ucp_worker;
    void*                   request;
    test_req_t*             ctx;
    int                     index;
} UcxTransmitterSendContext_;
/**
 * Set an address for the server to listen on - INADDR_ANY on a well known port.
 */
void set_sock_addr(const char* address_str, int port, struct sockaddr_storage* saddr);

void ep_close(ucp_worker_h ucp_worker, ucp_ep_h ep, uint32_t mode);

gxf_result_t init_worker(ucp_context_h ucp_context, ucp_worker_h* ucp_worker);

ucs_memory_type_t ucx_mem_type(MemoryStorageType gxf_mem_type);

ucs_status_t request_wait(ucp_worker_h ucp_worker, void* request,
                                 test_req_t* ctx);

gxf_result_t request_finalize(ucp_worker_h ucp_worker, void* request,
                                           test_req_t* ctx);

ucs_status_t request_wait_once(ucp_worker_h ucp_worker, void* request,
                                 test_req_t* ctx);

ucs_status_t process_request(ucp_worker_h ucp_worker, void* req);

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_EXTENSIONS_UCX_UCX_COMMON_HPP
