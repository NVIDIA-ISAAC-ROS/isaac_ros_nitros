/*
* Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
*
* NVIDIA CORPORATION and its licensors retain all intellectual property
* and proprietary rights in and to this software, related documentation
* and any modifications thereto.  Any use, reproduction, disclosure or
* distribution of this software and related documentation without an express
* license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#ifndef __APRILTAGS__
#define __APRILTAGS__

#include <stdint.h>
#include <stddef.h>
#include <vector>
#include <vector_types.h>

// Forward declaration for CUDA API
// CUstream and cudaStream_t are CUstream_st*
struct CUstream_st;

// Decoded AprilTag
typedef struct nvAprilTagsID_st
{
    float2 corners[4];
    uint16_t id;
    uint8_t hamming_error;
    float orientation[9];   //!< Rotation transform, when expressed as a 3x3 matrix acting on a column vector, is column major.
    float translation[3];   //!< Translation vector from the camera, in the same units as used for the tag_size.
}nvAprilTagsID_t;

// Input data type for image buffer
typedef struct nvAprilTagsImageInput_st
{
    uchar3* dev_ptr;    //!< Device pointer to the buffer
    size_t pitch;       //!< Pitch in bytes
    uint16_t width;     //!< Width in pixels
    uint16_t height;    //!< Buffer height
}nvAprilTagsImageInput_t;

typedef struct nvAprilTagsCameraIntrinsics_st {
    float fx, fy, cx, cy;
}nvAprilTagsCameraIntrinsics_t;

typedef enum
{
    NVAT_TAG36H11,                 // Default, currently the only tag family supported
    NVAT_ENUM_SIZE = 0x7fffffff    // Force int32_t
}
nvAprilTagsFamily;

//! AprilTags Detector instance handle. Used to reference the detector after creation
typedef struct nvAprilTagsHandle_st* nvAprilTagsHandle;

#ifdef __cplusplus
extern "C" {
#endif
    // FUNCTION NAME:   nvCreateAprilTagsDetector
    //
    //! DESCRIPTION:    Creates and initializes an AprilTags detector instance that detects and decodes April tags
    //!
    //! \param [out]    hApriltags          Pointer to the handle of newly created AprilTags detector
    //! \param [in]     img_width           Width of images to be fed in to AprilTags detector
    //! \param [in]     img_height          Height of images to be fed in to AprilTags detector
    //! \param [in]     tag_family          Enum representing the Tag Family to be detected; default NVAT_TAG36H11.
    //! \param [in]     cam                 Camera intrinsic parameters, or NULL, if the orientation and translation are not desired.
    //! \param [in]     tag_dim             The linear dimension of the square tag. The translation will be expressed in the same units.
    //!
    //! \retval :: 0 - Success, else - Failure
    int nvCreateAprilTagsDetector(nvAprilTagsHandle* hApriltags,
        const uint32_t img_width, const uint32_t img_height,
        const nvAprilTagsFamily tag_family,
        const nvAprilTagsCameraIntrinsics_t *cam,
        float tag_dim);

    // FUNCTION NAME:   nvAprilTagsDetect
    //
    //! DESCRIPTION:    Runs the algorithms to detect potential April tags in the image and decodes valid April tags 
    //!
    //! \param [in]     hApriltags          AprilTags detector handle
    //! \param [in]     img_input           Input buffer containing the undistorted image on which to detect/decode April tags
    //! \param [out]    tags_out            C-array containing detected Tags, after detection and decoding
    //! \param [out]    num_tags            Number of tags detected
    //! \param [in]     max_tags            Maximum number of tags that can be returned, based on allocated size of tags_out array.
    //! \param [in]     input_stream        CUDA stream on which the computation is to occur, or 0 to use the default stream.
    //!
    //! \retval :: 0 - Success, else - Failure
    int nvAprilTagsDetect(nvAprilTagsHandle hApriltags, const nvAprilTagsImageInput_t *img_input, 
        nvAprilTagsID_t *tags_out, uint32_t *num_tags, const uint32_t max_tags,
        CUstream_st* input_stream);

    // FUNCTION NAME:   nvAprilTagsDestroy
    //
    //! DESCRIPTION:    Destroys an instance of AprilTags detector
    //!
    //! \param [in]     hApriltags            AprilTags detector handle to be destroyed
    //!
    //! \retval :: 0 - Success, else - Failure
    int nvAprilTagsDestroy(nvAprilTagsHandle hApriltags);

#ifdef __cplusplus
}
#endif

#endif //__APRILTAGS__
