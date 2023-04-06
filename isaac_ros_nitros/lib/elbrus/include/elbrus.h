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

#define ELBRUS_DEFAULT(x) = (x)
#else
#include <stdint.h>
#include <stddef.h>

#define ELBRUS_DEFAULT(x)
#endif

#ifdef _WIN32
    #ifdef ELBRUS_EXPORT
        #define ELBRUS_API __declspec(dllexport)
    #else
        #define ELBRUS_API __declspec(dllimport)
    #endif
#else
    #define ELBRUS_API __attribute__((visibility("default")))
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
#define ELBRUS_API_VERSION_MAJOR 10
#define ELBRUS_API_VERSION_MINOR 5

/*
 * Use this function to check the version of the library you are using.
 *
 * Any one of the pointers could be null.
 *
 */
ELBRUS_API
void ELBRUS_GetVersion(int32_t* major, int32_t* minor);

/*
 * Transformation from camera space to world space.
 * Rotation matrix is column-major.
 */   
struct ELBRUS_Pose
{
    float r[9];
    float t[3];
};

/*
 * This structure encapsulates an inertial measurement unit reading
 */
struct ELBRUS_ImuMeasurement
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
 *         radial = (1 + k1 * r^2 + k2 * r^4 + k3 * r^6)
 * 
 *                                  | 2 * p1 * xn * yn + p2 * (r^2 + 2 * xn^2) |
 *         tandential_distort_vec = |                                          |
 *                                  | p1 * (r^2 + 2 * yn^2) + 2 * p2 * xn * yn | 
 * 
 *   xn = x/z, yn = y/z
 *   r^2 = (xn)^2 + (yn)^2
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
 */
struct ELBRUS_Camera
{
    const char* distortion_model;
    const float* parameters;
    int32_t num_parameters;
    int32_t width;
    int32_t height;

    /* Transformation from the coordinate frame of the camera
     * to the coordinate frame of the rig.
     */
    struct ELBRUS_Pose pose;
};

struct ELBRUS_CameraRig
{
    const struct ELBRUS_Camera* cameras;
    int32_t num_cameras;
};

#define ELBRUS_SBA_ON_GPU  0 /*Run sparse bundle adjustment on GPU. This is a default option.*/
#define ELBRUS_SBA_ON_CPU  1 /*Run sparse bundle adjustment on CPU*/
#define ELBRUS_SBA_OFF  2    /*Disable sparse bundle adjustment*/

/*
 * Configuration parameters that affect the whole tracking session.
 */
struct ELBRUS_Configuration
{
    /* Enable internal pose prediction mechanism based on a kinematic model.
     *
     * If frame rate is high enough it improves tracking performance
     * and stability.
     *
     * Prediction passed into `ELBRUS_TrackStereoSync` overrides prediction
     * from the kinematic model.
     *
     * As a general rule it is better to use a pose prediction mechanism
     * tailored to a specific application. If you have an IMU, consider using
     * it to provide pose predictions to Elbrus.
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

    /* Enable sparse bundle adjustment. This option reduces the drift at the cost of increased
     * hardware utilization.
     */
    int32_t sba_mode;

    /* Enable fast and robust left-to-right tracking for rectified
     * cameras with principal points on the horizontal line.
     */
    int32_t horizontal_stereo_camera;

    /* If IMU present this is left camera to imu transformation.
     * vImu = imu_from_left * vLeft;
     *      vImu - vector in imu coordinate system
     *      vLeft - vector in left eye coordinate system
     */
    struct ELBRUS_Pose imu_from_left;


    /* Gravitational acceleration that is used for the IMU integration,
     * defined in meters per sec^2.
     * The vector is defined in the world coordinate system:
     *   Cameras are always looking in the negative z direction.
     *   Y is "up", X is from "left" to "right"
     * For example: <0, -9.81, 0> is good enough for the most applications. */
    float g[3];

    /*
     * Allow to call ELBRUS_GetLastLeftObservations
     */
    int32_t enable_observations_export;
    /*
     * Allow to call ELBRUS_GetLastLandmarks
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
     * ELBRUS_EnableReadingDataLayer(), ELBRUS_DisableReadingDataLayer()
     */
    int32_t enable_reading_slam_internals;

    /*
     * Set directory where the dump files will be saved: 
     *   stereo.edex - cameras and configuration
     *   cam0.00000.png, cam1.00000.png, ... - input images
     * example:
     *    cfg->debug_dump_directory = "/tmp/elbrus"
     */
    const char* debug_dump_directory;

    /*
     * Enable IMU integrator
     */
    int32_t enable_imu_integrator;
    /*
     * Planar constraints.
     * Slam poses will be modified so that the camera moves on a horizontal plane.
     * See ELBRUS_GetSlamPose()
     */
    int32_t planar_constraints;
};

struct ELBRUS_Tracker;
typedef struct ELBRUS_Tracker* ELBRUS_TrackerHandle;

/*
 * Pixels must be stored row-wise
 */
struct ELBRUS_Image
{
    const uint8_t* pixels;

    /* Elbrus preserves timestamps: pose timestamp will match image timestamp 
     * Time must be in nanoseconds.
     */
    int64_t timestamp_ns;

    /* image resolution must match what was provided in ELBRUS_Camera */
    int32_t width;
    int32_t height;

    /* index of the camera in the rig */
    int32_t camera_index;
};

struct ELBRUS_Observation
{
    int32_t id;
    float u;    // 0 <= u < image width
    float v;    // 0 <= v < image height
};

struct ELBRUS_ObservationVector
{
    uint32_t num;
    uint32_t max;    // size of pre-allocated observations
    struct ELBRUS_Observation *observations;
};

struct ELBRUS_Landmark
{
    int64_t id;
    // coordinates in the camera space
    float x;
    float y;
    float z;
};

struct ELBRUS_Gravity
{
    // coordinates in the left camera space
    float x;
    float y;
    float z;
};

struct ELBRUS_LandmarkVector
{
    uint32_t num;
    uint32_t max;    // size of pre-allocated landmarks
    struct ELBRUS_Landmark *landmarks;
};

/*
* See ELBRUS_PoseEstimate::vo_state
*/
#define ELBRUS_VO_TRACKER_STATE_UNKNOWN         0   // Unknown state
#define ELBRUS_VO_TRACKER_STATE_SUCCESS         1   // Successed
#define ELBRUS_VO_TRACKER_STATE_FAILED          2   // Failed
#define ELBRUS_VO_TRACKER_STATE_INVALIDATED     3   // Successed but invalidated by IMU

/*
* See ELBRUS_PoseEstimate::integrator_state
*/
#define ELBRUS_INTEGRATOR_STATE_UNKNOWN         0
#define ELBRUS_INTEGRATOR_STATE_STATIC          1   // Static
#define ELBRUS_INTEGRATOR_STATE_INERTIAL        2   // Inertial
#define ELBRUS_INTEGRATOR_STATE_IMU             3   // IMU

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
struct ELBRUS_PoseEstimate
{
    struct ELBRUS_Pose pose;

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
    See #define ELBRUS_VO_TRACKER_STATE_*
    */
    uint32_t vo_state;
    /*
    State of the integrator (IMU)
    See #define ELBRUS_INTEGRATOR_STATE_*
    */
    uint32_t integrator_state;
};

typedef uint32_t ELBRUS_Status;

/* Error codes are subject to change except for ELBRUS_SUCCESS
 * Under normal conditions you should only expect to see
 * `ELBRUS_SUCCESS` and `ELBRUS_TRACKING_LOST`.
 * All other error codes indicate unexpected errors.
 * In case of an unexpected error the tracker should be reinitialized.
 */
#define ELBRUS_SUCCESS 0
#define ELBRUS_TRACKING_LOST 1  // obsolete
#define ELBRUS_INVALID_ARG 2
#define ELBRUS_OUT_OF_MEMORY 3
#define ELBRUS_GENERIC_ERROR 4
#define ELBRUS_UNSUPPORTED_NUMBER_OF_CAMERAS 5
#define ELBRUS_SLAM_IS_NOT_INITIALIZED 6
#define ELBRUS_NOT_IMPLEMENTED 7
#define ELBRUS_READING_SLAM_INTERNALS_DISABLED 8

/*
 * Creates the default configuration
 */
ELBRUS_API
void ELBRUS_InitDefaultConfiguration(struct ELBRUS_Configuration *cfg);

/*
 * Use this to initialize Elbrus
 *
 * ELBRUS_TrackerHandle will remember number of cameras
 * from rig. Elbrus supports only Mono and Stereo rigs.
 */
ELBRUS_API
ELBRUS_Status ELBRUS_CreateTracker(ELBRUS_TrackerHandle* tracker,
                                   const struct ELBRUS_CameraRig* rig,
                                   const struct ELBRUS_Configuration* cfg);

/*
 * Release all resources owned by the tracker
 */
ELBRUS_API
void ELBRUS_DestroyTracker(ELBRUS_TrackerHandle tracker);

/*
 * If visual odometry loses camera position, it briefly continues execution
 * using user-provided IMU measurements, while trying to recover the position.
 * You can call this function several times between image acquisition.
 * The timestamp is in nanoseconds and should always increment.
 * This function returns false in a case of wrong timestamp order.
 */
ELBRUS_API
ELBRUS_Status ELBRUS_RegisterImuMeasurement(ELBRUS_TrackerHandle tracker,
                                            int64_t timestamp,
                                            const struct ELBRUS_ImuMeasurement* imu);

/*
 * Track current frame synchronously:
 * the function blocks until the tracker has computed a pose.
 *
 * By default, this function uses visual odometry to compute a pose. If visual
 * odometry tracker fails to compute a pose, the function returns the position
 * calculated from a user-provided (via ELBRUS_RegisterImuMeasurement) IMU data.
 * If after several calls of ELBRUS_Track the visual odometry tracker is not
 * recovered ELBRUS_TRACKING_LOST will be returned.
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
 * Image timestamps have to match. Elbrus will use timestamp
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
 * rig->num_cameras in ELBRUS_CreateTracker.
 */
ELBRUS_API
ELBRUS_Status ELBRUS_Track(ELBRUS_TrackerHandle tracker,
                           const struct ELBRUS_Image *images,
                           const struct ELBRUS_Pose* predicted_pose,
                           struct ELBRUS_PoseEstimate* pose_estimate);


/*
 * Get rig pose which was estimated by visual odometry.
 * Call ELBRUS_Track() before ELBRUS_GetOdometryPose().
 * On success `pose` contains rig pose estimated by visual odometry
 *
 */
ELBRUS_API
ELBRUS_Status ELBRUS_GetOdometryPose(ELBRUS_TrackerHandle tracker,
    struct ELBRUS_Pose* pose_odom);

/*
 * Rig pose estimate by the slam system.
 * See ELBRUS_GetSlamPose().
 */
struct ELBRUS_PoseSlam
{
    struct ELBRUS_Pose pose;
};

/*
 * Get rig pose which was estimated by SLAM.
 * Call ELBRUS_Track() before ELBRUS_GetSlamPose().
 * On success `pose_slam->pose` contains rig pose estimated by slam 
 * 
 * You should set enable_localization_n_mapping=1 in the paramerets of ELBRUS_CreateTracker() 
 * else ELBRUS_SLAM_IS_NOT_INITIALIZED will be returned
 * 
 */
ELBRUS_API
ELBRUS_Status ELBRUS_GetSlamPose(ELBRUS_TrackerHandle tracker,
                                 struct ELBRUS_PoseSlam* pose_slam);

/*
 * Get list of poses for each frame.
 * poses_stamped will be filled. 
 * max_poses_stamped_count shows size of poses_stamped array.
 * 
 * Returns count of items was copied to poses_stamped array.
 */
struct ELBRUS_PoseStamped {
    int64_t timestamp_ns;
    struct ELBRUS_Pose pose;
};
ELBRUS_API
uint32_t ELBRUS_GetAllPoses(ELBRUS_TrackerHandle tracker,
    uint32_t max_poses_stamped_count,
    struct ELBRUS_PoseStamped* poses_stamped);


// Asynchronous response for ELBRUS_SaveToSlamDb()
typedef void(*ELBRUS_SaveToSlamDbResponse)(void* response_context, ELBRUS_Status status);

/*
 * Save Slam DB (map) to folder.
 * This folder will be created, if doesn`t exist.
 */
ELBRUS_API
ELBRUS_Status ELBRUS_SaveToSlamDb(
    /* Tracker handle, returned by ELBRUS_CreateTracker function. */
    ELBRUS_TrackerHandle tracker_handle,

    /* Folder name, where Slam lmdb database (map) will be saved. */
    const char* foldername,

    /* User defined asynchronous response, which will be called before the end of saving routine.
     * May be used to handle various error codes. */
    ELBRUS_SaveToSlamDbResponse response ELBRUS_DEFAULT(nullptr),

    /* Pointer to the response context, which will be passed to asynchronous response as argument. */
    void* response_context ELBRUS_DEFAULT(nullptr)
    );


// Asynchronous response for ELBRUS_LocalizeInExistDb()
typedef void(*ELBRUS_LocalizeInExistDbResponse)(void* response_context, ELBRUS_Status status, const struct ELBRUS_Pose* pose_in_db);

/*
 * Localize in the existing DB (map).
 * Finds the position of the camera in existing Slam lmdb database (map).
 * If success, moves the SLAM pose to the found position.
 * This is an asynchronous function. To receive result,
 * an asynchronous response ELBRUS_LocalizeInExistDbResponse is used.
 */
ELBRUS_API
ELBRUS_Status ELBRUS_LocalizeInExistDb(
    /* Tracker handle, returned by ELBRUS_CreateTracker function. */
    ELBRUS_TrackerHandle tracker_handle,

    /* Folder name, which stores saved Slam lmbd database (map). */
    const char* foldername,

    /* Pointer to the proposed pose, where the robot might be. */
    const struct ELBRUS_Pose* guess_pose,

    /* Radius of the area, where the robot might be. In meters. if 0, a default value will be used. */
    float radius ELBRUS_DEFAULT(0),

    /* Pointer the the observed images. Will be used if ELBRUS_Configuration.slam_sync_mode = 1.
     * Default value is nullptr. */
    const struct ELBRUS_Image* images ELBRUS_DEFAULT(nullptr),

    /* User defined asynchronous response, which will be called before the end of localization.
     * May be used to handle various error codes. */
    ELBRUS_LocalizeInExistDbResponse response ELBRUS_DEFAULT(nullptr),

    /* Pointer to the response context, which will be passed to asynchronous response as argument. */
    void* response_context ELBRUS_DEFAULT(nullptr)
    );

/*
 * Get current observations (SVIO 2d tracks) for visualization purpose.
 */
ELBRUS_API
ELBRUS_Status ELBRUS_GetLastLeftObservations(ELBRUS_TrackerHandle tracker,
                                             struct ELBRUS_ObservationVector *observations);

/*
 * Get current landmarks (SVIO 3d tracks) for visualization purpose.
 */
ELBRUS_API
ELBRUS_Status ELBRUS_GetLastLandmarks(ELBRUS_TrackerHandle tracker,
                                      struct ELBRUS_LandmarkVector *landmarks);

/*
 * Get gravity vector in the last VO frame
 */
ELBRUS_API
ELBRUS_Status ELBRUS_GetLastGravity(ELBRUS_TrackerHandle tracker, struct ELBRUS_Gravity *gravity);

/*
* Get internal slam metrics for visualization purpose.
*/
struct ELBRUS_SlamMetrics
{
    int64_t timestamp_ns;   // timestamp of these measurements (in nanoseconds)
    uint32_t lc_status;      // 0 - failed, 1 - succesed
    uint32_t pgo_status;     // 0 - failed, 1 - succesed
    uint32_t lc_selected_landmarks_count;   // Count of Landmarks Selected
    uint32_t lc_tracked_landmarks_count;    // Count of Landmarks Tracked
    uint32_t lc_pnp_landmarks_count;        // Count of Landmarks in PNP 
    uint32_t lc_good_landmarks_count;       // Count of Landmarks in LC
};

ELBRUS_API
ELBRUS_Status ELBRUS_GetSlamMetrics(ELBRUS_TrackerHandle tracker,
                                    struct ELBRUS_SlamMetrics* slam_metrics);




ELBRUS_API
ELBRUS_Status ELBRUS_EnablePoseAccumulator(ELBRUS_TrackerHandle tracker, uint32_t capacity);

/*
* Landmarks and pose graph reading
*/
struct ELBRUS_LandmarkInfo
{
    int64_t id;
    float weight;       
    // coordinates in the camera space
    float x;
    float y;
    float z;
};

struct ELBRUS_LandmarkInfoArrayRef
{
    uint64_t timestamp_ns;  // timestamp of landmarks
    uint32_t num;
    const struct ELBRUS_LandmarkInfo* landmarks;
};

struct ELBRUS_PoseGraphNode
{
    uint64_t id;
    struct ELBRUS_Pose node_pose;
};
struct ELBRUS_PoseGraphEdge
{
    uint64_t node_from;             // node id 
    uint64_t node_to;               // node id 
    struct ELBRUS_Pose transform;
    float covariance[6 * 6];
};
struct ELBRUS_PoseGraphRef
{
    uint64_t timestamp_ns;  // timestamp of pose graph
    uint32_t num_edges;
    uint32_t num_nodes;
    const struct ELBRUS_PoseGraphNode* nodes;
    const struct ELBRUS_PoseGraphEdge* edges;
};

struct ELBRUS_LocalizerProbe {
    uint64_t id;
    struct ELBRUS_Pose guess_pose;
    struct ELBRUS_Pose exact_result_pose;
    float weight;
    float exact_result_weight;
    int32_t solved;
};
struct ELBRUS_LocalizerProbesRef {
    uint64_t timestamp_ns;  // timestamp of pose graph
    uint32_t num_probes;
    float size;
    const struct ELBRUS_LocalizerProbe* probes;
};

enum ELBRUS_DataLayer {
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
ELBRUS_API ELBRUS_Status ELBRUS_EnableReadingDataLayer(
    ELBRUS_TrackerHandle tracker,
    enum ELBRUS_DataLayer layer,
    uint32_t max_items_count
    );
ELBRUS_API ELBRUS_Status ELBRUS_DisableReadingDataLayer(ELBRUS_TrackerHandle tracker, enum ELBRUS_DataLayer layer);

// Start landmarks layer reading. Have to call ELBRUS_FinishReadingLandmarks. Thread-safe. Lock free
// This function will fill all fields in the ELBRUS_LandmarkInfoArrayRef structure
ELBRUS_API ELBRUS_Status ELBRUS_StartReadingLandmarks(
    ELBRUS_TrackerHandle tracker, 
    enum ELBRUS_DataLayer layer,
    struct ELBRUS_LandmarkInfoArrayRef* landmarks     // output
    );
// Finish landmarks layer reading.
ELBRUS_API ELBRUS_Status ELBRUS_FinishReadingLandmarks(ELBRUS_TrackerHandle tracker, enum ELBRUS_DataLayer layer);

// Start pose graph reading. Have to call ELBRUS_FinishReadingPoseGraph. Thread-safe. Lock free
// This function will fill all fields in the ELBRUS_PoseGraphRef structure
ELBRUS_API ELBRUS_Status ELBRUS_StartReadingPoseGraph(
    ELBRUS_TrackerHandle tracker,
    enum ELBRUS_DataLayer layer,
    struct ELBRUS_PoseGraphRef* pose_graph     // output
    );
// Finish loop landmarks layer reading.
ELBRUS_API ELBRUS_Status ELBRUS_FinishReadingPoseGraph(ELBRUS_TrackerHandle tracker, enum ELBRUS_DataLayer layer);

// Start localizer probes reading. Have to call ELBRUS_FinishReadingPoseGraph. Thread-safe. Lock free
// This function will fill all fields in the ELBRUS_LocalizerProbesRef structure
ELBRUS_API ELBRUS_Status ELBRUS_StartReadingLocalizerProbes(
    ELBRUS_TrackerHandle tracker, 
    enum ELBRUS_DataLayer layer, 
    struct ELBRUS_LocalizerProbesRef* localizer_probes     // output
    );
ELBRUS_API ELBRUS_Status ELBRUS_FinishReadingLocalizerProbes(ELBRUS_TrackerHandle tracker, enum ELBRUS_DataLayer layer);

/*
* Set Log filename.
*/
ELBRUS_API
ELBRUS_Status ELBRUS_SetLogFilename(const char* filename);

#ifdef __cplusplus
} // extern "C"
#endif

