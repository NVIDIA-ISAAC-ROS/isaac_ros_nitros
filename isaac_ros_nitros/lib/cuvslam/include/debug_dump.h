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