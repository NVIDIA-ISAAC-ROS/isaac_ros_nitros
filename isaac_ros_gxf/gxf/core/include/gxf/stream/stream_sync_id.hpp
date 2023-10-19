// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#ifndef NVIDIA_GXF_STREAM_STREAM_SYNC_ID_HPP_
#define NVIDIA_GXF_STREAM_STREAM_SYNC_ID_HPP_

#include "gxf/core/gxf.h"


namespace nvidia {
namespace gxf {

// The Structure indicates stream sync component ID.
// Message entity carrying StreamSyncId indicates the ID of stream sync. The handle could
// be deduced by Handle<StreamSync>::Create(context, stream_sync_cid).
struct StreamSyncId {
  // component id of StreamSync
  gxf_uid_t stream_sync_cid;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_STREAM_STREAM_SYNC_ID_HPP_
