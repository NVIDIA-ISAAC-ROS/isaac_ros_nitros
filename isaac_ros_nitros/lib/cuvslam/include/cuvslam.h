/*
 * Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */
#pragma once

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

/*
 * None of the pointers are owning. Client is responsible for managing memory.
 */

/*
 * API is guaranteed to be compatible between the same major version numbers.
 */
#define CUVSLAM_API_VERSION_MAJOR 11
#define CUVSLAM_API_VERSION_MINOR 0

/*
 * Use this function to check the version of the library you are using.
 *
 * Any one of the pointers could be null.
 *
 */
CUVSLAM_API
void CUVSLAM_GetVersion(int32_t* major, int32_t* minor, const char** version);

/*
 * Transformation from camera space to world space.
 * Rotation matrix is column-major.
 */
struct CUVSLAM_Pose
{
    float r[9];
    float t[3];
};

struct CUVSLAM_ImuCalibration
{
    /* This is left camera from imu transformation.
     * vLeft = left_from_imu * vImu;
     *      vImu - vector in imu coordinate system
     *      vLeft - vector in left eye coordinate system
     */
    struct CUVSLAM_Pose left_from_imu;
    float gyroscope_noise_density;       // rad / (s * srqt(Hz))
    float gyroscope_random_walk;         // rad / (s ^ 2 * srqt(Hz))
    float accelerometer_noise_density;   // m / (s ^ 2 * srqt(Hz))
    float accelerometer_random_walk;     // m / (s ^ 3 * srqt(Hz))
    float frequency;                     // Hz
};

/*
 * This structure encapsulates an inertial measurement unit reading
 */
struct CUVSLAM_ImuMeasurement
{
    float linear_accelerations[3]; // in meters per squared second
    float angular_velocities[3];   // in radians per second
};

/*
 * Describes intrinsic and extrinsic parameters of a camera.
 *
 * Supported values of distortion_model:
 *
 * - brown5k
 *      9 parameters:
 *         0-1: principal point (cx, cy)
 *         2-3: focal length (fx, fy)
 *         4-6: radial distortion coeffs (k1, k2, k3)
 *         7-8: tangential distortion coeffs (p1, p2)
 *
 *      Each 3D point (x, y, z) is projected in the following way:
 *         (u, v) = (cx, cy) + diag(fx, fy) * (radial * (xn, yn) + tangential_distortion_vector)
 *      where:
 *          radial = (1 + k1 * r^2 + k2 * r^4 + k3 * r^6)
 *
 *                                   | 2 * p1 * xn * yn + p2 * (r^2 + 2 * xn^2) |
 *          tangential_distort_vec = |                                          |
 *                                   | p1 * (r^2 + 2 * yn^2) + 2 * p2 * xn * yn |
 *          xn = x/z, yn = y/z
 *          r^2 = (xn)^2 + (yn)^2
 *
 * - pinhole
 *      no distortion, same as radial5 with k0=k1=k2=p0=p1=0.f
 *      4 parameters:
 *         0-1: principal point (cx, cy)
 *         2-3: focal length (fx, fy)
 *
 * - fisheye4
 *      8 parameters:
 *         0-1: principal point (cx, cy)
 *         2-3: focal length (fx, fy)
 *         4-7: fisheye distortion coeffs (k1, k2, k3, k4)
 *      Each 3D point (x, y, z) is projected in the following way:
 *         (u, v) = (cx, cy) + diag(fx, fy) * (distorted_radius(r) * (xn, yn) / r)
 *      where:
 *          distorted_radius(r) = atan(r) * (1 + k1 * atan(r)^2 + k2 * atan(r)^4 + k3 * atan(r)^6 + k4 * atan(r)^8)
 *          xn = x/z, yn = y/z
 *          r^2 = (xn)^2 + (yn)^2
 *
 * - polynomial
 *      12 parameters:
 *         0-1: principal point (cx, cy)
 *         2-3: focal length (fx, fy)
 *         4-5: radial distortion coeffs (k1, k2)
 *         6-7: tangential distortion coeffs (p1, p2)
 *         8-11: radial distortion coeffs (k3, k4, k5, k6)
 *      Each 3D point (x, y, z) is projected in the following way:
 *         (u, v) = (cx, cy) + diag(fx, fy) * (radial * (xn, yn) + tangential_distortion_vector)
 *      where:
 *          radial = (1 + k1 * r^2 + k2 * r^4 + k3 * r^6) / (1 + k4 * r^2 + k5 * r^4 + k6 * r^6)
 *
 *                                   | 2 * p1 * xn * yn + p2 * (r^2 + 2 * xn^2) |
 *          tangential_distort_vec = |                                          |
 *                                   | p1 * (r^2 + 2 * yn^2) + 2 * p2 * xn * yn |
 *          xn = x/z, yn = y/z
 *          r^2 = (xn)^2 + (yn)^2
*/
struct CUVSLAM_Camera
{
    const char* distortion_model;
    const float* parameters;
    int32_t num_parameters;
    int32_t width;
    int32_t height;

    /* Transformation from the coordinate frame of the camera
     * to the coordinate frame of the rig.
     */
    struct CUVSLAM_Pose pose;
};

struct CUVSLAM_CameraRig
{
    const struct CUVSLAM_Camera* cameras;
    int32_t num_cameras;
};

/*
 * Configuration parameters that affect the whole tracking session.
 */
struct CUVSLAM_Configuration
{
    /* Enable internal pose prediction mechanism based on a kinematic model.
     *
     * If frame rate is high enough it improves tracking performance
     * and stability.
     *
     * Prediction passed into `CUVSLAM_TrackStereoSync` overrides prediction
     * from the kinematic model.
     *
     * As a general rule it is better to use a pose prediction mechanism
     * tailored to a specific application. If you have an IMU, consider using
     * it to provide pose predictions to cuVSLAM.
     *
     */
    int32_t use_motion_model;

    /* Enable image denoising.
     * Disable if the input images have already passed through a denoising
     * filter.
     */
    int32_t use_denoising;

    /* Enable feature tracking using GPU.
     */
    int32_t use_gpu;

    /* Enable fast and robust left-to-right tracking for rectified
     * cameras with principal points on the horizontal line.
     */
    int32_t horizontal_stereo_camera;

    /*
     * Allow to call CUVSLAM_GetLastLeftObservations
     */
    int32_t enable_observations_export;
    /*
     * Allow to call CUVSLAM_GetLastLandmarks
     */
    int32_t enable_landmarks_export;

    /*
     * Use localization and mapping
     */
    int32_t enable_localization_n_mapping;

    /*
     * Size of map cell. Default is 0 (the size will be calculated from the camera baseline)
     */
    float map_cell_size;

    /*
     * If localization and mapping is used:
     * sync mode (if true -> same thread with visual odometry). Default: slam_sync_mode = 0
     */
    int32_t slam_sync_mode;

    /*
     * Enable reading internal data from SLAM
     * CUVSLAM_EnableReadingDataLayer(), CUVSLAM_DisableReadingDataLayer()
     */
    int32_t enable_reading_slam_internals;

    /*
     * Set directory where the dump files will be saved:
     *   stereo.edex - cameras and configuration
     *   cam0.00000.png, cam1.00000.png, ... - input images
     * example:
     *    cfg->debug_dump_directory = "/tmp/cuvslam"
     */
    const char* debug_dump_directory;

    /*
     * Enable verbose flag for logging
     */
    int32_t verbosity;

    /*
     * Set maximum camera frame time in milliseconds.
     * Compares delta between frames in tracker with max_frame_delta_ms
     * to warn user and prevent mistakes depending on hardware settings.
     */
    float max_frame_delta_ms;


    struct CUVSLAM_ImuCalibration imu_calibration;

    /*
     * Enable IMU fusion
     */
    int32_t enable_imu_fusion;
    /*
     * Planar constraints.
     * Slam poses will be modified so that the camera moves on a horizontal plane.
     * See CUVSLAM_GetSlamPose()
     */
    int32_t planar_constraints;

    // Debug imu mode with only integration of rotation to check imu data correctness
    int32_t debug_imu_mode;
};

struct CUVSLAM_Tracker;
typedef struct CUVSLAM_Tracker* CUVSLAM_TrackerHandle;

/*
 * Pixels must be stored row-wise
 */
struct CUVSLAM_Image
{
    const uint8_t* pixels;

    /* cuVSLAM preserves timestamps: pose timestamp will match image timestamp
     * Time must be in nanoseconds.
     */
    int64_t timestamp_ns;

    /* image resolution must match what was provided in CUVSLAM_Camera */
    int32_t width;
    int32_t height;

    /* index of the camera in the rig */
    int32_t camera_index;
};

struct CUVSLAM_Observation
{
    int32_t id;
    float u;    // 0 <= u < image width
    float v;    // 0 <= v < image height
};

struct CUVSLAM_ObservationVector
{
    uint32_t num;
    uint32_t max;    // size of pre-allocated observations
    struct CUVSLAM_Observation *observations;
};

struct CUVSLAM_Landmark
{
    int64_t id;
    // coordinates in the camera space
    float x;
    float y;
    float z;
};

struct CUVSLAM_Gravity
{
    // coordinates in the left camera space
    float x;
    float y;
    float z;
};

struct CUVSLAM_LandmarkVector
{
    uint32_t num;
    uint32_t max;    // size of pre-allocated landmarks
    struct CUVSLAM_Landmark *landmarks;
};

/*
* See CUVSLAM_PoseEstimate::vo_state
*/
#define CUVSLAM_VO_TRACKER_STATE_UNKNOWN         0   // Unknown state
#define CUVSLAM_VO_TRACKER_STATE_SUCCESS         1   // Successed
#define CUVSLAM_VO_TRACKER_STATE_FAILED          2   // Failed
#define CUVSLAM_VO_TRACKER_STATE_INVALIDATED     3   // Successed but invalidated by IMU

/*
 * Rig pose estimate from the tracker.
 * All fields will be set by the tracker.
 *
 * Pose is a transformation from the rig coordinate space
 * to the world coordinate space.
 *
 * The rig coordinate space is user-defined and depends on the extrinsic
 * parameters of the cameras.
 *
 * Cameras are always looking in the negative z direction.
 *
 * The code has only been tested for the cases when one of the cameras'
 * coordinate spaces matches the rig coordinate space: extrinsic
 * parameters of the camera are the identity matrix.
 *
 * The world coordinate space is an arbitrary 3D coordinate frame.
 *
 * Pose covariance is defined via matrix exponential:
 * for a random zero-mean perturbation `u` in the tangent space
 * random pose is determined by `mean_pose * exp(u)`.
 */
struct CUVSLAM_PoseEstimate
{
    struct CUVSLAM_Pose pose;

    /* Pose timestamp in nanoseconds. */
    int64_t timestamp_ns;

    /* Row-major representation of the 6x6 covariance matrix
     * The orientation parameters use a fixed-axis representation.
     * In order, the parameters are:
     * (rotation about X axis, rotation about Y axis, rotation about Z axis, x, y, z)
     * Rotation in radians, translation in meters.
     */
    float covariance[6*6];

    /*
    State of vo tracking
    See #define CUVSLAM_VO_TRACKER_STATE_*
    */
    uint32_t vo_state;
};

typedef uint32_t CUVSLAM_Status;

/* Error codes are subject to change except for CUVSLAM_SUCCESS
 * Under normal conditions you should only expect to see
 * `CUVSLAM_SUCCESS` and `CUVSLAM_TRACKING_LOST`.
 * All other error codes indicate unexpected errors.
 * In case of an unexpected error the tracker should be reinitialized.
 */
#define CUVSLAM_SUCCESS 0
#define CUVSLAM_TRACKING_LOST 1
#define CUVSLAM_INVALID_ARG 2
#define CUVSLAM_OUT_OF_MEMORY 3
#define CUVSLAM_GENERIC_ERROR 4
#define CUVSLAM_UNSUPPORTED_NUMBER_OF_CAMERAS 5
#define CUVSLAM_SLAM_IS_NOT_INITIALIZED 6
#define CUVSLAM_NOT_IMPLEMENTED 7
#define CUVSLAM_READING_SLAM_INTERNALS_DISABLED 8

/*
 * Warms up GPU, creates CUDA runtime context.
 *
 * This function is not mandatory to call, but helps to save some time in tracker initialization.
 */
CUVSLAM_API
void CUVSLAM_WarmUpGPU();

/*
 * Creates the default configuration
 */
CUVSLAM_API
void CUVSLAM_InitDefaultConfiguration(struct CUVSLAM_Configuration *cfg);

/*
 * Use this to initialize cuVSLAM
 *
 * CUVSLAM_TrackerHandle will remember number of cameras
 * from rig. cuVSLAM supports only Mono and Stereo rigs.
 */
CUVSLAM_API
CUVSLAM_Status CUVSLAM_CreateTracker(CUVSLAM_TrackerHandle* tracker,
                                   const struct CUVSLAM_CameraRig* rig,
                                   const struct CUVSLAM_Configuration* cfg);

/*
 * Release all resources owned by the tracker
 */
CUVSLAM_API
void CUVSLAM_DestroyTracker(CUVSLAM_TrackerHandle tracker);

/*
 * If visual odometry loses camera position, it briefly continues execution
 * using user-provided IMU measurements, while trying to recover the position.
 * You can call this function several times between image acquisition.
 * The timestamp is in nanoseconds and should always increment.
 * This function returns false in a case of wrong timestamp order.
 */
CUVSLAM_API
CUVSLAM_Status CUVSLAM_RegisterImuMeasurement(CUVSLAM_TrackerHandle tracker,
                                            int64_t timestamp,
                                            const struct CUVSLAM_ImuMeasurement* imu);

/*
 * Track current frame synchronously:
 * the function blocks until the tracker has computed a pose.
 *
 * By default, this function uses visual odometry to compute a pose. If visual
 * odometry tracker fails to compute a pose, the function returns the position
 * calculated from a user-provided (via CUVSLAM_RegisterImuMeasurement) IMU data.
 * If after several calls of CUVSLAM_Track the visual odometry tracker is not
 * recovered CUVSLAM_TRACKING_LOST will be returned.
 *
 * If `predicted_pose` is not NULL, the tracker will use it
 * as the initial guess.
 *
 * The track will output poses in the same coordinate system
 * until a loss of tracking.
 *
 * On success `pose_estimate` contains estimated rig pose.
 * On failure value of `pose_estimate` is undefined.
 *
 * Image timestamps have to match. cuVSLAM will use timestamp
 * of the image taken with camera 0.
 *
 * If your camera rig provides "almost synchronized" frames,
 * you could use one of the following for the common timestamp:
 * - timestamp from camera 0
 * - average timestamp
 *
 * images - is a pointer to single image in case of mono or
 * array of two images in case of stereo.
 *
 * You should use the same number of images for tracker equal to
 * rig->num_cameras in CUVSLAM_CreateTracker.
 */
CUVSLAM_API
CUVSLAM_Status CUVSLAM_Track(CUVSLAM_TrackerHandle tracker,
                           const struct CUVSLAM_Image *images,
                           const struct CUVSLAM_Pose* predicted_pose,
                           struct CUVSLAM_PoseEstimate* pose_estimate);


/*
 * Get rig pose which was estimated by visual odometry.
 * Call CUVSLAM_Track() before CUVSLAM_GetOdometryPose().
 * On success `pose` contains rig pose estimated by visual odometry
 *
 */
CUVSLAM_API
CUVSLAM_Status CUVSLAM_GetOdometryPose(CUVSLAM_TrackerHandle tracker,
    struct CUVSLAM_Pose* pose_odom);

/*
 * Rig pose estimate by the slam system.
 * See CUVSLAM_GetSlamPose().
 */
struct CUVSLAM_PoseSlam
{
    struct CUVSLAM_Pose pose;
};

/*
 * Get rig pose which was estimated by SLAM.
 * Call CUVSLAM_Track() before CUVSLAM_GetSlamPose().
 * On success `pose_slam->pose` contains rig pose estimated by slam
 *
 * You should set enable_localization_n_mapping=1 in the paramerets of CUVSLAM_CreateTracker()
 * else CUVSLAM_SLAM_IS_NOT_INITIALIZED will be returned
 *
 */
CUVSLAM_API
CUVSLAM_Status CUVSLAM_GetSlamPose(CUVSLAM_TrackerHandle tracker,
                                 struct CUVSLAM_PoseSlam* pose_slam);

/*
 * Get list of poses for each frame.
 * poses_stamped will be filled.
 * max_poses_stamped_count shows size of poses_stamped array.
 *
 * Returns count of items was copied to poses_stamped array.
 */
struct CUVSLAM_PoseStamped {
    int64_t timestamp_ns;
    struct CUVSLAM_Pose pose;
};
CUVSLAM_API
uint32_t CUVSLAM_GetAllPoses(CUVSLAM_TrackerHandle tracker,
    uint32_t max_poses_stamped_count,
    struct CUVSLAM_PoseStamped* poses_stamped);


// Asynchronous response for CUVSLAM_SaveToSlamDb()
typedef void(*CUVSLAM_SaveToSlamDbResponse)(void* response_context, CUVSLAM_Status status);

/*
 * Save Slam DB (map) to folder.
 * This folder will be created, if doesn`t exist.
 */
CUVSLAM_API
CUVSLAM_Status CUVSLAM_SaveToSlamDb(
    /* Tracker handle, returned by CUVSLAM_CreateTracker function. */
    CUVSLAM_TrackerHandle tracker_handle,

    /* Folder name, where Slam lmdb database (map) will be saved. */
    const char* foldername,

    /* User defined asynchronous response, which will be called before the end of saving routine.
     * May be used to handle various error codes. */
    CUVSLAM_SaveToSlamDbResponse response CUVSLAM_DEFAULT(nullptr),

    /* Pointer to the response context, which will be passed to asynchronous response as argument. */
    void* response_context CUVSLAM_DEFAULT(nullptr)
    );


// Asynchronous response for CUVSLAM_LocalizeInExistDb()
typedef void(*CUVSLAM_LocalizeInExistDbResponse)(void* response_context, CUVSLAM_Status status, const struct CUVSLAM_Pose* pose_in_db);

/*
 * Localize in the existing DB (map).
 * Finds the position of the camera in existing Slam lmdb database (map).
 * If success, moves the SLAM pose to the found position.
 * This is an asynchronous function. To receive result,
 * an asynchronous response CUVSLAM_LocalizeInExistDbResponse is used.
 */
CUVSLAM_API
CUVSLAM_Status CUVSLAM_LocalizeInExistDb(
    /* Tracker handle, returned by CUVSLAM_CreateTracker function. */
    CUVSLAM_TrackerHandle tracker_handle,

    /* Folder name, which stores saved Slam lmbd database (map). */
    const char* foldername,

    /* Pointer to the proposed pose, where the robot might be. */
    const struct CUVSLAM_Pose* guess_pose,

    /* Radius of the area, where the robot might be. In meters. if 0, a default value will be used. */
    float radius CUVSLAM_DEFAULT(0),

    /* Pointer the the observed images. Will be used if CUVSLAM_Configuration.slam_sync_mode = 1.
     * Default value is nullptr. */
    const struct CUVSLAM_Image* images CUVSLAM_DEFAULT(nullptr),

    /* User defined asynchronous response, which will be called before the end of localization.
     * May be used to handle various error codes. */
    CUVSLAM_LocalizeInExistDbResponse response CUVSLAM_DEFAULT(nullptr),

    /* Pointer to the response context, which will be passed to asynchronous response as argument. */
    void* response_context CUVSLAM_DEFAULT(nullptr)
    );

/*
 * Get current observations (SVIO 2d tracks) for visualization purpose.
 */
CUVSLAM_API
CUVSLAM_Status CUVSLAM_GetLastLeftObservations(CUVSLAM_TrackerHandle tracker,
                                             struct CUVSLAM_ObservationVector *observations);

/*
 * Get current landmarks (SVIO 3d tracks) for visualization purpose.
 */
CUVSLAM_API
CUVSLAM_Status CUVSLAM_GetLastLandmarks(CUVSLAM_TrackerHandle tracker,
                                      struct CUVSLAM_LandmarkVector *landmarks);

/*
 * Get gravity vector in the last VO frame
 */
CUVSLAM_API
CUVSLAM_Status CUVSLAM_GetLastGravity(CUVSLAM_TrackerHandle tracker, struct CUVSLAM_Gravity *gravity);

/*
* Get internal slam metrics for visualization purpose.
*/
struct CUVSLAM_SlamMetrics
{
    int64_t timestamp_ns;   // timestamp of these measurements (in nanoseconds)
    uint32_t lc_status;      // 0 - failed, 1 - succesed
    uint32_t pgo_status;     // 0 - failed, 1 - succesed
    uint32_t lc_selected_landmarks_count;   // Count of Landmarks Selected
    uint32_t lc_tracked_landmarks_count;    // Count of Landmarks Tracked
    uint32_t lc_pnp_landmarks_count;        // Count of Landmarks in PNP
    uint32_t lc_good_landmarks_count;       // Count of Landmarks in LC
};

CUVSLAM_API
CUVSLAM_Status CUVSLAM_GetSlamMetrics(CUVSLAM_TrackerHandle tracker,
                                    struct CUVSLAM_SlamMetrics* slam_metrics);




CUVSLAM_API
CUVSLAM_Status CUVSLAM_EnablePoseAccumulator(CUVSLAM_TrackerHandle tracker, uint32_t capacity);

/*
* Landmarks and pose graph reading
*/
struct CUVSLAM_LandmarkInfo
{
    int64_t id;
    float weight;
    // coordinates in the camera space
    float x;
    float y;
    float z;
};

struct CUVSLAM_LandmarkInfoArrayRef
{
    uint64_t timestamp_ns;  // timestamp of landmarks
    uint32_t num;
    const struct CUVSLAM_LandmarkInfo* landmarks;
};

struct CUVSLAM_PoseGraphNode
{
    uint64_t id;
    struct CUVSLAM_Pose node_pose;
};
struct CUVSLAM_PoseGraphEdge
{
    uint64_t node_from;             // node id
    uint64_t node_to;               // node id
    struct CUVSLAM_Pose transform;
    float covariance[6 * 6];
};
struct CUVSLAM_PoseGraphRef
{
    uint64_t timestamp_ns;  // timestamp of pose graph
    uint32_t num_edges;
    uint32_t num_nodes;
    const struct CUVSLAM_PoseGraphNode* nodes;
    const struct CUVSLAM_PoseGraphEdge* edges;
};

struct CUVSLAM_LocalizerProbe {
    uint64_t id;
    struct CUVSLAM_Pose guess_pose;
    struct CUVSLAM_Pose exact_result_pose;
    float weight;
    float exact_result_weight;
    int32_t solved;
};
struct CUVSLAM_LocalizerProbesRef {
    uint64_t timestamp_ns;  // timestamp of pose graph
    uint32_t num_probes;
    float size;
    const struct CUVSLAM_LocalizerProbe* probes;
};

enum CUVSLAM_DataLayer {
    LL_OBSERVATIONS = 0,        // Landmarks cloud. Landmarks that are operated in current frame
    LL_MAP = 1,                 // Landmarks cloud. Landmarks of the map
    LL_LOOP_CLOSURE = 2,        // Landmarks cloud. Map's landmarks that are visible in the last loop closure event
    LL_POSE_GRAPH = 3,          // Pose Graph
    LL_LOCALIZER_PROBES = 4,    // Localizer probes
    LL_LOCALIZER_MAP = 5,       // Landmarks cloud. Landmarks of the Localizer map (opened database)
    LL_LOCALIZER_OBSERVATIONS = 6,   // Landmarks cloud. Landmarks that are visible in the localization
    LL_LOCALIZER_LOOP_CLOSURE = 7,   // Landmarks cloud. Landmarks that are visible in the final loop closure of the localization
    LL_MAX = 8
};

// Enable or disable landmarks layer reading.
CUVSLAM_API CUVSLAM_Status CUVSLAM_EnableReadingDataLayer(
    CUVSLAM_TrackerHandle tracker,
    enum CUVSLAM_DataLayer layer,
    uint32_t max_items_count
    );
CUVSLAM_API CUVSLAM_Status CUVSLAM_DisableReadingDataLayer(CUVSLAM_TrackerHandle tracker, enum CUVSLAM_DataLayer layer);

// Start landmarks layer reading. Have to call CUVSLAM_FinishReadingLandmarks. Thread-safe. Lock free
// This function will fill all fields in the CUVSLAM_LandmarkInfoArrayRef structure
CUVSLAM_API CUVSLAM_Status CUVSLAM_StartReadingLandmarks(
    CUVSLAM_TrackerHandle tracker,
    enum CUVSLAM_DataLayer layer,
    struct CUVSLAM_LandmarkInfoArrayRef* landmarks     // output
    );
// Finish landmarks layer reading.
CUVSLAM_API CUVSLAM_Status CUVSLAM_FinishReadingLandmarks(CUVSLAM_TrackerHandle tracker, enum CUVSLAM_DataLayer layer);

// Start pose graph reading. Have to call CUVSLAM_FinishReadingPoseGraph. Thread-safe. Lock free
// This function will fill all fields in the CUVSLAM_PoseGraphRef structure
CUVSLAM_API CUVSLAM_Status CUVSLAM_StartReadingPoseGraph(
    CUVSLAM_TrackerHandle tracker,
    enum CUVSLAM_DataLayer layer,
    struct CUVSLAM_PoseGraphRef* pose_graph     // output
    );
// Finish loop landmarks layer reading.
CUVSLAM_API CUVSLAM_Status CUVSLAM_FinishReadingPoseGraph(CUVSLAM_TrackerHandle tracker, enum CUVSLAM_DataLayer layer);

// Start localizer probes reading. Have to call CUVSLAM_FinishReadingPoseGraph. Thread-safe. Lock free
// This function will fill all fields in the CUVSLAM_LocalizerProbesRef structure
CUVSLAM_API CUVSLAM_Status CUVSLAM_StartReadingLocalizerProbes(
    CUVSLAM_TrackerHandle tracker,
    enum CUVSLAM_DataLayer layer,
    struct CUVSLAM_LocalizerProbesRef* localizer_probes     // output
    );
CUVSLAM_API CUVSLAM_Status CUVSLAM_FinishReadingLocalizerProbes(CUVSLAM_TrackerHandle tracker, enum CUVSLAM_DataLayer layer);

/*
* Set Log filename.
*/
CUVSLAM_API
CUVSLAM_Status CUVSLAM_SetLogFilename(const char* filename);

#ifdef __cplusplus
} // extern "C"
#endif

