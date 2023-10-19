// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

/// @cond Doxygen_Suppress
#ifdef __cplusplus
#include <cstdint>
#include <cstddef>

#define CUVSLAM_DEFAULT(x) = (x)
#else
#include <stdint.h>
#include <stddef.h>

#define CUVSLAM_DEFAULT(x)
#endif

#ifdef _WIN32
#ifdef CUVSLAM_EXPORT
#define CUVSLAM_API __declspec(dllexport)
#else
#define CUVSLAM_API __declspec(dllimport)
#endif
#else
#define CUVSLAM_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif
/// @endcond

/// Major version. API is guaranteed to be compatible between the same major version numbers.
#define CUVSLAM_API_VERSION_MAJOR 11

/// Minor version
#define CUVSLAM_API_VERSION_MINOR 4

/**
 * @name Error codes
 * Error codes are subject to change except for CUVSLAM_SUCCESS.
 * Under normal conditions you should only expect to see
 * ::CUVSLAM_SUCCESS and ::CUVSLAM_TRACKING_LOST.
 * All other error codes indicate unexpected errors.
 * In case of an unexpected error the tracker should be reinitialized.
 */
///@{

/// Success
#define CUVSLAM_SUCCESS 0

/// Tracking lost
#define CUVSLAM_TRACKING_LOST 1

/// Invalid argument
#define CUVSLAM_INVALID_ARG 2

/// Can't localize
#define CUVSLAM_CAN_NOT_LOCALIZE 3

/// Generic error
#define CUVSLAM_GENERIC_ERROR 4

/// Unsupported number of cameras
#define CUVSLAM_UNSUPPORTED_NUMBER_OF_CAMERAS 5

/// Slam is not initialized
#define CUVSLAM_SLAM_IS_NOT_INITIALIZED 6

/// Not implemented
#define CUVSLAM_NOT_IMPLEMENTED 7

/// Reading slam internals is disabled
#define CUVSLAM_READING_SLAM_INTERNALS_DISABLED 8
///@}

/**
 * @name CUVSLAM_VO_TRACKER_STATE
 *  @see CUVSLAM_PoseEstimate::vo_state
 */
///@{

/// Unknown
#define CUVSLAM_VO_TRACKER_STATE_UNKNOWN         0

/// Successes
#define CUVSLAM_VO_TRACKER_STATE_SUCCESS         1

/// Failed
#define CUVSLAM_VO_TRACKER_STATE_FAILED          2

///@}

/**
 * Data layer
 */
enum CUVSLAM_DataLayer {
    LL_OBSERVATIONS = 0,           ///< Landmarks that are operated in current frame
    LL_MAP = 1,                    ///< Landmarks of the map
    LL_LOOP_CLOSURE = 2,           ///< Map's landmarks that are visible in the last loop closure event
    LL_POSE_GRAPH = 3,             ///< Pose Graph
    LL_LOCALIZER_PROBES = 4,       ///< Localizer probes
    LL_LOCALIZER_MAP = 5,          ///< Landmarks of the Localizer map (opened database)
    LL_LOCALIZER_OBSERVATIONS = 6, ///< Landmarks that are visible in the localization
    LL_LOCALIZER_LOOP_CLOSURE = 7, ///< Landmarks that are visible in the final loop closure of the localization
    LL_MAX = 8
};

/**
 * Transformation from camera space to world space
 */
struct CUVSLAM_Pose {
    float r[9];  ///< rotation column-major matrix
    float t[3];  ///< translation vector
};

/**
 * IMU Calibration
 */
struct CUVSLAM_ImuCalibration {
    struct CUVSLAM_Pose left_from_imu;  /**< Left camera from imu transformation.
                                             vLeft = left_from_imu * vImu
                                             - vImu - vector in imu coordinate system
                                             - vLeft - vector in left eye coordinate system */
    float gyroscope_noise_density;      ///< \f$rad / (s * \sqrt{hz})\f$
    float gyroscope_random_walk;        ///< \f$rad / (s^2 * \sqrt{hz})\f$
    float accelerometer_noise_density;  ///< \f$m / (s^2 * \sqrt{hz})\f$
    float accelerometer_random_walk;    ///< \f$m / (s^3 * \sqrt{hz})\f$
    float frequency;                    ///< \f$hz\f$
};

/**
 * Inertial measurement unit reading
 */
struct CUVSLAM_ImuMeasurement {
    float linear_accelerations[3]; ///< in meters per squared second
    float angular_velocities[3];   ///< in radians per second
};

/**
 * Describes intrinsic and extrinsic parameters of a camera.
 * Supported values of distortion_model:
 * - brown5k (9 parameters):
 *   * 0-1: principal point \f$(c_x, c_y)\f$\n
 *   * 2-3: focal length \f$(f_x, f_y)\f$\n
 *   * 4-6: radial distortion coefficients \f$(k_1, k_2, k_3)\f$\n
 *   * 7-8: tangential distortion coefficients \f$(p_1, p_2)\f$\n
 *   .
 *   Each 3D point \f$(x, y, z)\f$ is projected in the following way:\n
 *      \f$(u, v) = (c_x, c_y) + diag(f_x, f_y) * (radial * (x_n, y_n) + tangentialDistortion)\f$\n
 *   where:\n
 *      \f$radial = (1 + k_1 * r^2 + k_2 * r^4 + k_3 * r^6)\f$\n
 *      \f$tangentialDistortion.x = 2 * p_1 * x_n * y_n + p_2 * (r^2 + 2 * x_n^2)\f$\n
 *      \f$tangentialDistortion.y = p_1 * (r^2 + 2 * y_n^2) + 2 * p_2 * x_n * y_n\f$\n
 *      \f$x_n = \frac{x}{z}\f$\n
 *      \f$y_n = \frac{y}{z}\f$\n
 *      \f$r = \sqrt{(x_n)^2 + (y_n)^2}\f$\n
 * - pinhole (4 parameters):\n
 *   no distortion, same as radial5 with \f$k_0=k_1=k_2=p_0=p_1=0\f$\n
 *   * 0-1: principal point \f$(c_x, c_y)\f$\n
 *   * 2-3: focal length \f$(f_x, f_y)\f$\n
 * - fisheye4 (8 parameters):\n
 *   * 0-1: principal point \f$(c_x, c_y)\f$\n
 *   * 2-3: focal length \f$(f_x, f_y)\f$\n
 *   * 4-7: fisheye distortion coefficients \f$(k_1, k_2, k_3, k_4)\f$\n
 *   .
 *   Each 3D point \f$(x, y, z)\f$ is projected in the following way:\n
 *   \f$(u, v) = (c_x, c_y) + diag(f_x, f_y) * (distortedR(r) * (x_n, y_n) / r)\f$\n
 *   where:\n
 *     \f$distortedR(r) = \arctan(r) * (1 + k_1 * \arctan^2(r) + k_2 * \arctan^4(r) + k_3 * \arctan^6(r) + k_4 * \arctan^8(r))\f$\n
 *      \f$x_n = \frac{x}{z}\f$\n
 *      \f$y_n = \frac{y}{z}\f$\n
 *      \f$r = \sqrt{(x_n)^2 + (y_n)^2}\f$\n
 * - polynomial (12 parameters):\n
 *   * 0-1: principal point \f$(c_x, c_y)\f$\n
 *   * 2-3: focal length \f$(f_x, f_y)\f$\n
 *   * 4-5: radial distortion coefficients \f$(k_1, k_2)\f$\n
 *   * 6-7: tangential distortion coefficients \f$(p_1, p_2)\f$\n
 *   * 8-11: radial distortion coefficients \f$(k_3, k_4, k_5, k_6)\f$\n
 *   .
 *   Each 3D point \f$(x, y, z)\f$ is projected in the following way:\n
 *     \f$(u, v) = (c_x, c_y) + diag(f_x, f_y) * (radial * (x_n, y_n) + tangentialDistortion)\f$\n
 *   where:\n
 *      \f$radial = \frac{1 + k_1 * r^2 + k_2 * r^4 + k_3 * r^6}{1 + k_4 * r^2 + k_5 * r^4 + k_6 * r^6}\f$\n
 *      \f$tangentialDistortion.x = 2 * p_1 * x_n * y_n + p_2 * (r^2 + 2 * x_n^2)\f$\n
 *      \f$tangentialDistortion.y = p_1 * (r^2 + 2 * y_n^2) + 2 * p_2 * x_n * y_n\f$\n
 *      \f$x_n = \frac{x}{z}\f$\n
 *      \f$y_n = \frac{y}{z}\f$\n
 *      \f$r = \sqrt{(x_n)^2 + (y_n)^2}\f$\n
*/
struct CUVSLAM_Camera {
    const char *distortion_model;  ///< "polynomial" or "fisheye4" or "pinhole" or "brown5k"
    const float *parameters;       ///< list of parameters
    int32_t num_parameters;        ///< parameters number
    int32_t width;                 ///< image width in pixels
    int32_t height;                ///< image height in pixels
    struct CUVSLAM_Pose pose;      ///< transformation from coordinate frame of the camera to frame of the rig
};

/**
 * Camera rig
 */
struct CUVSLAM_CameraRig {
    const struct CUVSLAM_Camera *cameras;   ///< list of cameras parameters
    int32_t num_cameras;                    ///< number of cameras (should be 2 for stereo)
};

/**
 * Configuration parameters that affect the whole tracking session
 */
struct CUVSLAM_Configuration {
    /**
     * Enable internal pose prediction mechanism based on a kinematic model.
     * If frame rate is high enough it improves tracking performance
     * and stability.
     * Prediction passed into `CUVSLAM_TrackStereoSync` overrides prediction
     * from the kinematic model.
     * As a general rule it is better to use a pose prediction mechanism
     * tailored to a specific application. If you have an IMU, consider using
     * it to provide pose predictions to cuVSLAM.
     *
     */
    int32_t use_motion_model;

    /**
     * Enable image denoising. Disable if the input images have already passed through a denoising filter.
     */
    int32_t use_denoising;

    /**
     * Enable feature tracking using GPU.
     */
    int32_t use_gpu;

    /**
     * Enable fast and robust left-to-right tracking for rectified cameras with principal points on the horizontal line.
     */
    int32_t horizontal_stereo_camera;

    /**
     * Allow to call CUVSLAM_GetLastLeftObservations.
     */
    int32_t enable_observations_export;

    /**
     * Allow to call CUVSLAM_GetLastLandmarks.
     */
    int32_t enable_landmarks_export;

    /**
     * Use localization and mapping.
     */
    int32_t enable_localization_n_mapping;

    /**
     * Size of map cell. Default is 0 (the size will be calculated from the camera baseline).
     */
    float map_cell_size;

    /**
     * If localization and mapping is used:
     * sync mode (if true -> same thread with visual odometry). Default: slam_sync_mode = 0
     */
    int32_t slam_sync_mode;

    /**
     * Enable reading internal data from SLAM
     * CUVSLAM_EnableReadingDataLayer(), CUVSLAM_DisableReadingDataLayer()
     */
    int32_t enable_reading_slam_internals;

    /**
     * Set directory where the dump files will be saved:
     *   stereo.edex - cameras and configuration
     *   cam0.00000.png, cam1.00000.png, ... - input images
     * example:
     *    cfg->debug_dump_directory = "/tmp/cuvslam"
     */
    const char *debug_dump_directory;

    /**
     * Enable verbose flag for logging
     */
    int32_t verbosity;

    /**
     * Set maximum camera frame time in milliseconds.
     * Compares delta between frames in tracker with max_frame_delta_ms
     * to warn user and prevent mistakes depending on hardware settings.
     */
    float max_frame_delta_ms;

    /**
     * IMU calibration
     */
    struct CUVSLAM_ImuCalibration imu_calibration;

    /**
     * Enable IMU fusion
     */
    int32_t enable_imu_fusion;

    /**
     * Planar constraints.
     * Slam poses will be modified so that the camera moves on a horizontal plane.
     * See CUVSLAM_GetSlamPose()
     */
    int32_t planar_constraints;

    /**
     * Debug imu mode with only integration of rotation to check imu data correctness
     */
    int32_t debug_imu_mode;

    /** It's a special slam mode for visual map building in case ground truth is present.
     *  Not realtime, no loop closure, no map global optimization, SBA in main thread
     * Requirements:
     *      enable_localization_n_mapping = true
     *      slam_sync_mode = 1
     *      planar_constraints = 0
     * Default is off (=0)
     */
    int32_t slam_gt_align_mode;
};

/**
 * Tracker
 */
struct CUVSLAM_Tracker;

/**
 * Tracker Handle
 */
typedef struct CUVSLAM_Tracker *CUVSLAM_TrackerHandle;

/**
 * Image
 */
struct CUVSLAM_Image {
    const uint8_t *pixels; ///< Pixels must be stored row-wise
    int64_t timestamp_ns;  ///< pose timestamp will match image timestamp. Time must be in nanoseconds.
    int32_t width;         ///< image width must match what was provided in CUVSLAM_Camera
    int32_t height;        ///< image height must match what was provided in CUVSLAM_Camera
    int32_t camera_index;  ///< index of the camera in the rig
};

/**
 * Observation
 */
struct CUVSLAM_Observation {
    int32_t id; ///< id
    float u;    ///< 0 <= u < image width
    float v;    ///< 0 <= v < image height
};

/**
 * Observations list
 */
struct CUVSLAM_ObservationVector {
    uint32_t num;                               ///< number of filled elements in observations list
    uint32_t max;                               ///< size of pre-allocated observations
    struct CUVSLAM_Observation *observations;   ///< observations
};

/**
 * Landmark
 * x, y, z in camera frame
 */
struct CUVSLAM_Landmark {
    int64_t id;  ///< identifier
    float x;     ///< x - coordinates
    float y;     ///< y - coordinates
    float z;     ///< z - coordinates
};

/**
 * List of landmarks
 */
struct CUVSLAM_LandmarkVector {
    uint32_t num;                        ///< number of filled elements in landmarks list
    uint32_t max;                        ///< size of pre-allocated landmarks
    struct CUVSLAM_Landmark *landmarks;  ///< landmarks
};

/**
 * Gravity vector in the left camera space
 */
struct CUVSLAM_Gravity {
    float x;    ///< x - coordinate in meters
    float y;    ///< y - coordinate in meters
    float z;    ///< z - coordinate in meters
};

/**
 * Rig pose estimate from the tracker.
 * All fields will be set by the tracker.
 * The rig coordinate space is user-defined and depends on the extrinsic
 * parameters of the cameras.
 * Cameras are always looking in the negative z direction.
 * The code has only been tested for the cases when one of the cameras'
 * coordinate spaces matches the rig coordinate space: extrinsic
 * parameters of the camera are the identity matrix.
 * The world coordinate space is an arbitrary 3D coordinate frame.
 * Pose covariance is defined via matrix exponential:
 * for a random zero-mean perturbation `u` in the tangent space
 * random pose is determined by `mean_pose * exp(u)`.
 */
struct CUVSLAM_PoseEstimate {
    /**
     * Transformation from the rig coordinate space to the world coordinate space.
     */
    struct CUVSLAM_Pose pose;

    /**
     * Pose timestamp in nanoseconds
     */
    int64_t timestamp_ns;

    /** Row-major representation of the 6x6 covariance matrix
     * The orientation parameters use a fixed-axis representation.
     * In order, the parameters are:
     * (rotation about X axis, rotation about Y axis, rotation about Z axis, x, y, z)
     * Rotation in radians, translation in meters.
     */
    float covariance[6 * 6];

    /**
     * State of vo tracking @see CUVSLAM_VO_TRACKER_STATE_*
     */
    uint32_t vo_state;
};

/**
 * Call result code
 */
typedef uint32_t CUVSLAM_Status;

/**
 * Get internal slam metrics for visualization purpose.
 */
struct CUVSLAM_SlamMetrics {
    int64_t timestamp_ns;                   ///< timestamp of these measurements (in nanoseconds)
    uint32_t lc_status;                     ///< 0 - fail, 1 - success
    uint32_t pgo_status;                    ///< 0 - fail, 1 - success
    uint32_t lc_selected_landmarks_count;   ///< Count of Landmarks Selected
    uint32_t lc_tracked_landmarks_count;    ///< Count of Landmarks Tracked
    uint32_t lc_pnp_landmarks_count;        ///< Count of Landmarks in PNP
    uint32_t lc_good_landmarks_count;       ///< Count of Landmarks in LC
};

/**
 * Landmarks coordinates in the camera space and pose graph reading
 */
struct CUVSLAM_LandmarkInfo {
    int64_t id;     ///< identifier
    float weight;   ///< weight
    float x;        ///< x - coordinate in meters
    float y;        ///< y - coordinate in meters
    float z;        ///< z - coordinate in meters
};

/**
 * List of landmarks
 */
struct CUVSLAM_LandmarkInfoArrayRef {
    uint64_t timestamp_ns;                          ///< timestamp of landmarks in nanoseconds
    uint32_t num;                                   ///< landmarks number
    const struct CUVSLAM_LandmarkInfo *landmarks;   ///< landmarks list
};

/**
 * Pose graph node
 */
struct CUVSLAM_PoseGraphNode {
    uint64_t id;                        ///< node identifier
    struct CUVSLAM_Pose node_pose;      ///< node pose
};

/**
 * Pose graph edge
 */
struct CUVSLAM_PoseGraphEdge {
    uint64_t node_from;             ///< node id
    uint64_t node_to;               ///< node id
    struct CUVSLAM_Pose transform;  ///< transform
    float covariance[6 * 6];        ///< covariance
};

/**
 * Pose graph
 */
struct CUVSLAM_PoseGraphRef {
    uint64_t timestamp_ns;                      ///< timestamp of pose graph in nanoseconds
    uint32_t num_edges;                         ///< edges number
    uint32_t num_nodes;                         ///< nodes number
    const struct CUVSLAM_PoseGraphNode *nodes;  ///< nodes list
    const struct CUVSLAM_PoseGraphEdge *edges;  ///< edges list
};

/**
 * Localizer probes
 */
struct CUVSLAM_LocalizerProbe {
    uint64_t id;                            ///< probe identifier
    struct CUVSLAM_Pose guess_pose;         ///< input hint
    struct CUVSLAM_Pose exact_result_pose;  ///< exact pose if localizer success
    float weight;                           ///< input wight
    float exact_result_weight;              ///< result weight
    int32_t solved;                         ///< 1 for solved, 0 for unsolved
};

/**
 * List of localizer probes
 */
struct CUVSLAM_LocalizerProbesRef {
    uint64_t timestamp_ns;                          ///< timestamp of localizer try in nanoseconds
    uint32_t num_probes;                            ///< number of probes
    float size;                                     ///< size of search area
    const struct CUVSLAM_LocalizerProbe *probes;    ///< list of probes
};

/**
 * Pose with timestamp
 */
struct CUVSLAM_PoseStamped {
    int64_t timestamp_ns;       ///< timestamp
    struct CUVSLAM_Pose pose;   ///<
};

/**
 * Asynchronous response for CUVSLAM_SaveToSlamDb()
 * @param[in] response_context - context
 * @param[in] status           - save result
 */
typedef void(*CUVSLAM_SaveToSlamDbResponse)(void *response_context, CUVSLAM_Status status);

/**
 * Asynchronous response for CUVSLAM_LocalizeInExistDb()
 * @param[in] response_context - context
 * @param[in] status           - localize status
 * @param[in] pose_in_db       - position in database
 */
typedef void(*CUVSLAM_LocalizeInExistDbResponse)(void *response_context, CUVSLAM_Status status,
                                                 const struct CUVSLAM_Pose *pose_in_db);

/**
 * Use this function to check the version of the library you are using.
 * Any one of the pointers could be null.
 * @param[out] major   - major version
 * @param[out] minor   - minor version
 * @param[out] version - detailed version in string format
 */
CUVSLAM_API
void CUVSLAM_GetVersion(int32_t *major, int32_t *minor, const char **version);

/**
 * Set Log filename. For internal usage only.
 * @param[in] filename new log filename
 * @return result status (error code)
 */
CUVSLAM_API
CUVSLAM_Status CUVSLAM_SetLogFilename(const char *filename);

/**
 * Warms up GPU, creates CUDA runtime context.
 * This function is not mandatory to call, but helps to save some time in tracker initialization.
 */
CUVSLAM_API
void CUVSLAM_WarmUpGPU();

/**
 * Creates the default configuration
 * @param[out] cfg
 */
CUVSLAM_API
void CUVSLAM_InitDefaultConfiguration(struct CUVSLAM_Configuration *cfg);

/**
 * Use this to initialize cuVSLAM
 * CUVSLAM_TrackerHandle will remember number of cameras
 * from rig. cuVSLAM supports only Mono and Stereo rigs.
 * @param[out] tracker created tracker handle
 * @param[in] rig      rig setup
 * @param[in] cfg      tracker configuration
 * @return result status (error code)
 */
CUVSLAM_API
CUVSLAM_Status CUVSLAM_CreateTracker(CUVSLAM_TrackerHandle *tracker, const struct CUVSLAM_CameraRig *rig,
                                     const struct CUVSLAM_Configuration *cfg);

/**
 * Release all resources owned by the tracker
 * @param[in] tracker tracker handle
 */
CUVSLAM_API
void CUVSLAM_DestroyTracker(CUVSLAM_TrackerHandle tracker);

/**
 * If visual odometry loses camera position, it briefly continues execution
 * using user-provided IMU measurements while trying to recover the position.
 * You should call this function several times between image acquisition.
 *
 * - CUVSLAM_Track
 * - CUVSLAM_RegisterImuMeasurement
 * - ...
 * - CUVSLAM_RegisterImuMeasurement
 * - CUVSLAM_Track
 *
 * Imu measurement and frame image both have timestamps, so it is important to call these functions in
 * strict timestamps in ascending order. CUVSLAM_RegisterImuMeasurement is a thread-safe so you can call
 * CUVSLAM_RegisterImuMeasurement and CUVSLAM_Track simultaneously.
 *
 * @param[in] tracker   tracker handle
 * @param[in] timestamp timestamp is in nanoseconds and should always increment
 * @param[in] imu       IMU measurements
 * @return result status (error code). This function returns false in a case of wrong timestamp order.
 * @see CUVSLAM_Track
 */
CUVSLAM_API
CUVSLAM_Status CUVSLAM_RegisterImuMeasurement(CUVSLAM_TrackerHandle tracker, int64_t timestamp,
                                              const struct CUVSLAM_ImuMeasurement *imu);

/**
 * Track current frame synchronously:
 * the function blocks until the tracker has computed a pose.
 * By default, this function uses visual odometry to compute a pose. If visual
 * odometry tracker fails to compute a pose, the function returns the position
 * calculated from a user-provided IMU data.
 * If after several calls of CUVSLAM_Track the visual odometry tracker is not
 * recovered CUVSLAM_TRACKING_LOST will be returned.
 * The track will output poses in the same coordinate system
 * until a loss of tracking.
 * Image timestamps have to match. cuVSLAM will use timestamp
 * of the image taken with camera 0.
 * If your camera rig provides "almost synchronized" frames,
 * you could use one of the following for the common timestamp:
 * - timestamp from camera 0
 * - average timestamp
 * You should use the same number of images for tracker equal to
 * rig->num_cameras in CUVSLAM_CreateTracker.
 * @param[in]  tracker        tracker handle
 * @param[in]  images         is a pointer to single image in case of mono or array of two images in case of stereo
 * @param[in]  predicted_pose If `predicted_pose` is not NULL, the tracker will use it as the initial guess.
 *                            For slam_gt_align_mode predicted pose is the ground truth pose.
 * @param[out] pose_estimate  On success `pose_estimate` contains estimated rig pose.
 *                            On failure value of `pose_estimate` is undefined.
 * @return result status (error code)
 * @see CUVSLAM_RegisterImuMeasurement
 */
CUVSLAM_API
CUVSLAM_Status CUVSLAM_Track(CUVSLAM_TrackerHandle tracker, const struct CUVSLAM_Image *images,
                             const struct CUVSLAM_Pose *predicted_pose, struct CUVSLAM_PoseEstimate *pose_estimate);

/**
 * Get rig pose which was estimated by visual odometry.
 * Call CUVSLAM_Track() before CUVSLAM_GetOdometryPose().
 * @param[in]  tracker tracker handle
 * @param[out] pose    on success contains rig pose estimated by visual odometry
 * @return result status (error code)
 */
CUVSLAM_API
CUVSLAM_Status CUVSLAM_GetOdometryPose(CUVSLAM_TrackerHandle tracker, struct CUVSLAM_Pose *pose);

/**
 * Get rig pose which was estimated by SLAM.
 * Call CUVSLAM_Track() before CUVSLAM_GetSlamPose().
 * You should set enable_localization_n_mapping=1 in the parameters of CUVSLAM_CreateTracker()
 * else CUVSLAM_SLAM_IS_NOT_INITIALIZED will be returned
 * @param[in]  tracker tracker handle
 * @param[out] pose    on success contains rig pose estimated by slam
 * @return result status (error code)
 */
CUVSLAM_API
CUVSLAM_Status CUVSLAM_GetSlamPose(CUVSLAM_TrackerHandle tracker, struct CUVSLAM_Pose *pose);

/**
 * Get list of poses for each frame.
 * @param[in]      tracker                  tracker handle
 * @param[in]      max_poses_stamped_count  size of poses_stamped array
 * @param[in, out] poses_stamped            pre-allocated array to store poses
 * @return number of items copied to poses_stamped array
 */
CUVSLAM_API
uint32_t CUVSLAM_GetAllPoses(CUVSLAM_TrackerHandle tracker, uint32_t max_poses_stamped_count,
                             struct CUVSLAM_PoseStamped *poses_stamped);

/**
 * Save Slam DB (map) to folder.
 * This folder will be created, if does not exist.
 * @param[in] tracker           tracker handle
 * @param[in] folder_name       Folder name, where Slam lmdb database (map) will be saved.
 * @param[in] response          User defined asynchronous response, which will be called before the end of
 *                              saving routine. May be used to handle various error codes.
 * @param[in] response_context  Pointer to the response context, which will be passed to
 *                              asynchronous response as argument.
 * @return result status (error code)
 */
CUVSLAM_API
CUVSLAM_Status CUVSLAM_SaveToSlamDb(CUVSLAM_TrackerHandle tracker, const char *folder_name,
                                    CUVSLAM_SaveToSlamDbResponse response CUVSLAM_DEFAULT(nullptr),
                                    void *response_context CUVSLAM_DEFAULT(nullptr));

/**
 * Localize in the existing DB (map).
 * Finds the position of the camera in existing Slam lmdb database (map).
 * If success, moves the SLAM pose to the found position.
 * This is an asynchronous function. To receive result,
 * an asynchronous response CUVSLAM_LocalizeInExistDbResponse is used.
 * @param[in] tracker           tracker handle
 * @param[in] folder_name       Folder name, which stores saved Slam database (map).
 * @param[in] guess_pose        Pointer to the proposed pose, where the robot might be.
 * @param[in] radius            Radius in meters of the area, where the robot might be.
 *                              If 0, a default value will be used.
 * @param[in] images            Observed images. Will be used if CUVSLAM_Configuration.slam_sync_mode = 1.
 * @param[in] response          User defined asynchronous response, which will be called before the end of localization.
 *                              May be used to handle various error codes.
 * @param[in] response_context  Pointer to the response context, which will be passed to asynchronous
 *                              response as argument.
 * @return result status (error code)
 */
CUVSLAM_API
CUVSLAM_Status CUVSLAM_LocalizeInExistDb(CUVSLAM_TrackerHandle tracker, const char *folder_name,
                                         const struct CUVSLAM_Pose *guess_pose, float radius CUVSLAM_DEFAULT(0),
                                         const struct CUVSLAM_Image *images CUVSLAM_DEFAULT(nullptr),
                                         CUVSLAM_LocalizeInExistDbResponse response CUVSLAM_DEFAULT(nullptr),
                                         void *response_context CUVSLAM_DEFAULT(nullptr));

/**
 * Get current observations (visual odometry 2d tracks) for visualization purpose.
 * @param[in]  tracker      tracker handle
 * @param[out] observations observations list
 * @return result status (error code)
 */
CUVSLAM_API
CUVSLAM_Status CUVSLAM_GetLastLeftObservations(CUVSLAM_TrackerHandle tracker,
                                               struct CUVSLAM_ObservationVector *observations);

/**
 * Get current landmarks (visual odometry 3d tracks) for visualization purpose.
 * @param[in]  tracker   tracker handle
 * @param[out] landmarks landmarks list
 * @return result status (error code)
 */
CUVSLAM_API
CUVSLAM_Status CUVSLAM_GetLastLandmarks(CUVSLAM_TrackerHandle tracker, struct CUVSLAM_LandmarkVector *landmarks);

/**
 * Get gravity vector in the last VO frame
 * @param[in]  tracker tracker handle
 * @param[out] gravity gravity vector
 * @return result status (error code)
 */
CUVSLAM_API
CUVSLAM_Status CUVSLAM_GetLastGravity(CUVSLAM_TrackerHandle tracker, struct CUVSLAM_Gravity *gravity);

/**
 * Get slam metrics
 * @param[in]  tracker      tracker handle
 * @param[out] slam_metrics slam info
 * @return result status (error code)
 */
CUVSLAM_API
CUVSLAM_Status CUVSLAM_GetSlamMetrics(CUVSLAM_TrackerHandle tracker, struct CUVSLAM_SlamMetrics *slam_metrics);

/**
 * Enable or disable landmarks layer reading
 * @param[in] tracker           tracker handle
 * @param[in] layer             any layer is acceptable
 * @param[in] max_items_count   maximum items number
 * @return result status (error code)
 */
CUVSLAM_API
CUVSLAM_Status CUVSLAM_EnableReadingDataLayer(CUVSLAM_TrackerHandle tracker, enum CUVSLAM_DataLayer layer,
                                              uint32_t max_items_count);

/**
 * Disable reading data layer
 * @param[in] tracker tracker handle
 * @param[in] layer   any layer is acceptable
 * @return result status (error code)
 */
CUVSLAM_API
CUVSLAM_Status CUVSLAM_DisableReadingDataLayer(CUVSLAM_TrackerHandle tracker, enum CUVSLAM_DataLayer layer);

/**
 * Start landmarks layer reading. Have to call CUVSLAM_FinishReadingLandmarks. Thread-safe. Lock free
 * This function will fill all fields in the CUVSLAM_LandmarkInfoArrayRef structure
 * @param[in]  tracker    tracker handle
 * @param[in]  layer      one from LL_OBSERVATIONS, LL_MAP, LL_LOOP_CLOSURE, LL_LOCALIZER_MAP,
 *                        LL_LOCALIZER_OBSERVATIONS, LL_LOCALIZER_LOOP_CLOSURE
 * @param[out] landmarks  landmarks list
 * @return result status (error code)
 */
CUVSLAM_API
CUVSLAM_Status CUVSLAM_StartReadingLandmarks(CUVSLAM_TrackerHandle tracker, enum CUVSLAM_DataLayer layer,
                                             struct CUVSLAM_LandmarkInfoArrayRef *landmarks);

/**
 * Finish landmarks layer reading
 * @param[in] tracker tracker handle
 * @param[in] layer   one from LL_OBSERVATIONS, LL_MAP, LL_LOOP_CLOSURE, LL_LOCALIZER_MAP,
 *                    LL_LOCALIZER_OBSERVATIONS, LL_LOCALIZER_LOOP_CLOSURE
 * @return result status (error code)
 */
CUVSLAM_API
CUVSLAM_Status CUVSLAM_FinishReadingLandmarks(CUVSLAM_TrackerHandle tracker, enum CUVSLAM_DataLayer layer);

/**
 * Start pose graph reading. Have to call CUVSLAM_FinishReadingPoseGraph. Thread-safe. Lock free
 * This function will fill all fields in the CUVSLAM_PoseGraphRef structure
 * @param[in]  tracker    tracker handle
 * @param[out] pose_graph pose graph
 * @return result status (error code)
 */
CUVSLAM_API
CUVSLAM_Status CUVSLAM_StartReadingPoseGraph(CUVSLAM_TrackerHandle tracker, struct CUVSLAM_PoseGraphRef *pose_graph);

/**
 * Finish loop landmarks layer reading
 * @param[in] tracker tracker handle
 * @return result status (error code)
 */
CUVSLAM_API
CUVSLAM_Status CUVSLAM_FinishReadingPoseGraph(CUVSLAM_TrackerHandle tracker);

/**
 * Start localizer probes reading. Have to call CUVSLAM_FinishReadingPoseGraph. Thread-safe. Lock free
 * This function will fill all fields in the CUVSLAM_LocalizerProbesRef structure
 * @param[in]  tracker           tracker handle
 * @param[out] localizer_probes  list of localizer probes
 * @return result status (error code)
 */
CUVSLAM_API
CUVSLAM_Status CUVSLAM_StartReadingLocalizerProbes(CUVSLAM_TrackerHandle tracker,
                                                   struct CUVSLAM_LocalizerProbesRef *localizer_probes);

/**
 * Finish reading localizer probes
 * @param[in] tracker tracker handle
 * @return result status (error code)
 */
CUVSLAM_API
CUVSLAM_Status CUVSLAM_FinishReadingLocalizerProbes(CUVSLAM_TrackerHandle tracker);


/**
 * Merge existing maps into one map
 * @param[in] databases input array of directories with existing dbs
 * @param[in] databases_num size of input array
 * @param[in] output_folder directory to save output db
 * @return result status (error code)
 */
CUVSLAM_API
CUVSLAM_Status CUVSLAM_MergeDatabases(char const *const *databases, int32_t databases_num, char const *output_folder);

#ifdef __cplusplus
} // extern "C"
#endif
