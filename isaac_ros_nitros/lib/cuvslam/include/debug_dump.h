// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#pragma once

#include "cuvslam.h"

#ifdef __cplusplus
extern "C" {
#endif
/// @endcond

CUVSLAM_API
void DumpcuVSLAMConfiguration(
    const char* input_dump_root_dir,
    const struct CUVSLAM_CameraRig *rig,
    const struct CUVSLAM_Configuration *cfg);

CUVSLAM_API
void DumpcuVSLAMTrackCall(
    const char* input_dump_root_dir,
    size_t frame_id,
    const struct CUVSLAM_Image *images,
    size_t num_images,
    bool gpu_images);

CUVSLAM_API
void DumpcuVSLAMRegisterImuMeasurementCall(
    const char* input_dump_root_dir,
    int64_t time_ns,
    const struct CUVSLAM_ImuMeasurement& data);

#ifdef __cplusplus
}
#endif