/*
 * Copyright (c) 2026, NVIDIA CORPORATION. All rights reserved.
 *
 * NVIDIA software released under the NVIDIA Open Software License is intended to be used permissively and enable the
 * further development of AI technologies. Subject to the terms of this License, NVIDIA confirms that you are free to
 * commercially use, modify, and distribute the software with NVIDIA hardware. NVIDIA does not claim ownership to any
 * outputs generated using the software or derivative works thereof. By using, reproducing, modifying, distributing,
 * performing or displaying any portion or element of the software or derivative works thereof, you agree to be bound by
 * this License.
 */

#pragma once

#include <array>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

/// @cond Doxygen_Suppress
#ifdef _WIN32
#ifdef CUVSLAM_EXPORT
#define CUVSLAM_API __declspec(dllexport)
#else
#define CUVSLAM_API __declspec(dllimport)
#endif
#else
#define CUVSLAM_API __attribute__((visibility("default")))
#endif
/// @endcond

namespace cuvslam {

/**
 * @brief Get the version of the library.
 * Any one of the pointers could be null.
 * @param[out] major   - major version
 * @param[out] minor   - minor version
 * @param[out] patch   - patch version
 * @return semantic version string view
 */
CUVSLAM_API
std::string_view GetVersion(int32_t* major, int32_t* minor, int32_t* patch);

/**
 * Set verbosity. The higher the value, the more output from the library. 0 (default) for no output.
 * @param[in] verbosity new verbosity value
 */
CUVSLAM_API
void SetVerbosity(int verbosity);

/**
 * Warms up GPU, creates CUDA runtime context.
 * This function is not mandatory to call, but helps to save some time in tracker initialization.
 * It can also be used to quickly diagnose issues with CUDA or CUDA libraries.
 * @throws std::runtime_error if CUDA, cusolver or cublas initialization fails.
 */
CUVSLAM_API
void WarmUpGPU();

/**
 * Static-size array of 32-bit floats
 */
template <std::size_t N>
using Array = std::array<float, N>;

/**
 * 3D vector of floats
 */
using Vector3f = Array<3>;

/**
 * Static-size array of 32-bit integers
 */
template <std::size_t N>
using IntArray = std::array<int32_t, N>;

/**
 * Transformation from one frame to another.
 * cuVSLAM uses OpenCV coordinate system convention: x is right, y is down, z is forward.
 */
struct Pose {
  Array<4> rotation = {0, 0, 0, 1};  ///< rotation quaternion in (x, y, z, w) order
  Array<3> translation = {0, 0, 0};  ///< translation vector
};

/**
 * 6x6 covariance matrix
 */
using PoseCovariance = Array<6 * 6>;

/**
 * @brief Distortion model with parameters
 *
 * Terminology:
 * - principal point \f$(c_x, c_y)\f$
 * - focal length \f$(f_x, f_y)\f$
 * - 2x2 diagonal matrix \f$\mathrm{diag}(f_x, f_y) = \begin{bmatrix} f_x & 0 \\ 0 & f_y \end{bmatrix}\f$
 *
 * Supported distortion models:
 *
 * - Pinhole (0 parameters)
 *   - No distortion; equivalent to Brown with \f$k_0=k_1=k_2=p_0=p_1=0\f$.
 *
 * - Fisheye (4 parameters)
 *   - Also known as equidistant model for pinhole cameras.
 *   - Coefficients \f$k_1, k_2, k_3, k_4\f$ are compatible with ethz-asl/kalibr (pinhole-equi) and OpenCV::fisheye.
 *   - Limitation: this (pinhole + undistort) approach works only for FOV < 180°. TUMVI has ~190°.
 *     EuRoC and ORB_SLAM3 use a different approach (direct project/unproject without pinhole) and support > 180°;
 *     their coefficients are incompatible with this model.
 *   - Parameters:
 *     - 0..3: \f$(k_1, k_2, k_3, k_4)\f$
 *   - Projection:
 *     - \f$(u, v) = (c_x, c_y) + \mathrm{diag}(f_x, f_y) \cdot \frac{\mathrm{radial}(r) \cdot (x_n, y_n)}{r}\f$
 *     - where:
 *       - \f$\mathrm{radial}(r) = \arctan(r) \cdot \left(1 + k_1 \arctan^2(r) + k_2 \arctan^4(r) + k_3 \arctan^6(r)
 * + k_4 \arctan^8(r)\right)\f$
 *       - \f$x_n = x/z\f$
 *       - \f$y_n = y/z\f$
 *       - \f$r = \sqrt{x_n^2 + y_n^2}\f$
 *
 * - Brown (5 parameters)
 *   - Equivalent to Polynomial model with \f$k_4=k_5=k_6=0\f$; \b note a different order of parameters.
 *   - Parameters:
 *     - 0..2: radial \f$(k_1, k_2, k_3)\f$
 *     - 3..4: tangential \f$(p_1, p_2)\f$
 *   - Projection:
 *     - \f$(u, v) = (c_x, c_y) + \mathrm{diag}(f_x, f_y) \cdot \left( \mathrm{radial} \cdot (x_n, y_n) + (t_x, t_y)
 * \right)\f$
 *     - where:
 *       - \f$\mathrm{radial} = 1 + k_1 r^2 + k_2 r^4 + k_3 r^6\f$
 *       - \f$t_x = 2 p_1 x_n y_n + p_2 (r^2 + 2 x_n^2)\f$
 *       - \f$t_y = p_1 (r^2 + 2 y_n^2) + 2 p_2 x_n y_n\f$
 *       - \f$x_n = x/z\f$
 *       - \f$y_n = y/z\f$
 *       - \f$r = \sqrt{x_n^2 + y_n^2}\f$
 *
 * - Polynomial (8 parameters)
 *   - Coefficients are compatible with the first 8 coefficients of the OpenCV distortion model.
 *   - Parameters:
 *     - 0..1: radial \f$(k_1, k_2)\f$
 *     - 2..3: tangential \f$(p_1, p_2)\f$
 *     - 4..7: radial \f$(k_3, k_4, k_5, k_6)\f$
 *   - Projection:
 *     - \f$(u, v) = (c_x, c_y) + \mathrm{diag}(f_x, f_y) \cdot \left( \mathrm{radial} \cdot (x_n, y_n) + (t_x, t_y)
 * \right)\f$
 *     - where:
 *       - \f$\mathrm{radial} = \frac{1 + k_1 r^2 + k_2 r^4 + k_3 r^6}{1 + k_4 r^2 + k_5 r^4 + k_6 r^6}\f$
 *       - \f$t_x = 2 p_1 x_n y_n + p_2 (r^2 + 2 x_n^2)\f$
 *       - \f$t_y = p_1 (r^2 + 2 y_n^2) + 2 p_2 x_n y_n\f$
 *       - \f$x_n = x/z\f$
 *       - \f$y_n = y/z\f$
 *       - \f$r = \sqrt{x_n^2 + y_n^2}\f$
 */
struct Distortion {
  /**
   * Distortion model type
   */
  enum class Model : uint8_t {
    Pinhole,
    Fisheye,
    Brown,
    Polynomial,
  };

  Model model = Model::Pinhole;   ///< distortion model @see Model
  std::vector<float> parameters;  ///< array of distortion parameters depending on model
};

/**
 * @brief Camera parameters
 *
 * Describes intrinsic and extrinsic parameters of a camera and per-camera settings.
 *
 * For camera coordinate system top left pixel has (0, 0) coordinate (y is down, x is right).
 * It's compatible with ROS CameraInfo/OpenCV.
 */
struct Camera {
  IntArray<2> size;          ///< image size in pixels (width, height)
  Array<2> principal;        ///< principal point in pixels \f$(c_x, c_y)\f$
  Array<2> focal;            ///< focal length in pixels \f$(f_x, f_y)\f$
  Pose rig_from_camera;      ///< transformation from coordinate frame of the camera to frame of the rig
  Distortion distortion;     ///< distortion parameters
  int32_t border_top{0};     ///< offset from the top border where visual features will be ignored (default: 0)
  int32_t border_bottom{0};  ///< offset from the bottom border where visual features will be ignored (default: 0)
  int32_t border_left{0};    ///< offset from the left border where visual features will be ignored (default: 0)
  int32_t border_right{0};   ///< offset from the right border where visual features will be ignored (default: 0)
};

/**
 * @brief IMU Calibration parameters
 *
 * Describes intrinsic and extrinsic (noise and random walk) parameters of an IMU sensor.
 * See [IMU Noise Model](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model)
 */
struct ImuCalibration {
  Pose rig_from_imu;                  /**< Rig from imu transformation.
                                           vRig = rig_from_imu * vImu
                                           - vImu - vector in imu coordinate system
                                           - vRig - vector in rig coordinate system */
  float gyroscope_noise_density;      ///< \f$rad / (s * \sqrt{hz})\f$
  float gyroscope_random_walk;        ///< \f$rad / (s^2 * \sqrt{hz})\f$
  float accelerometer_noise_density;  ///< \f$m / (s^2 * \sqrt{hz})\f$
  float accelerometer_random_walk;    ///< \f$m / (s^3 * \sqrt{hz})\f$
  float frequency;                    ///< \f$hz\f$
};

/**
 * @brief Rig consisting of cameras and IMU sensors
 *
 * @note 1 to 32 cameras are supported now.
 * @note 0 or 1 IMU sensor is supported now.
 * @note IMU sensor can be used only in Odometry::OdometryMode::Inertial mode with a single stereo camera.
 */
struct Rig {
  std::vector<Camera> cameras;       ///< Cameras; 1 to 32 cameras are supported now
  std::vector<ImuCalibration> imus;  ///< IMU sensors; 0 or 1 sensor is supported now
};

/**
 * @brief Image data structure
 *
 * @note Image pixels must be stored row-wise (right to left, top to bottom)
 * @note Image width and height must match Camera::size
 */
struct ImageData {
  /// @brief Image encoding
  enum class Encoding : uint8_t {
    MONO,  ///< grayscale or other single-channel data
    RGB,   ///< RGB
  };
  /// @brief Image data type
  enum class DataType : uint8_t {
    UINT8,    ///< 8-bit unsigned integer
    UINT16,   ///< 16-bit unsigned integer
    FLOAT32,  ///< 32-bit floating point
  };

  const void* pixels;  ///< Pixels must be stored row-wise (right to left, top to bottom)
  int32_t width;       ///< image width must match Camera::size
  int32_t height;      ///< image height must match Camera::size
  int32_t pitch;       ///< bytes per image row including padding for GPU memory images, ignored for CPU images
  Encoding encoding;   ///< grayscale and RGB are supported now
  DataType data_type;  ///< image data type
  bool is_gpu_mem;     ///< is pixels pointer points to GPU or CPU memory buffer
};

/**
 * @brief Image with timestamp and camera index
 */
struct Image : public ImageData {
  int64_t timestamp_ns;   ///< Image timestamp in nanoseconds
  uint32_t camera_index;  ///< index of the camera in the rig
};

/**
 * @brief IMU measurement
 */
struct ImuMeasurement {
  int64_t timestamp_ns;           ///< IMU measurement timestamp in nanoseconds
  Array<3> linear_accelerations;  ///<  \f$m / s^2\f$
  Array<3> angular_velocities;    ///< \f$rad / s\f$
};

/**
 * @brief Pose with timestamp
 */
struct PoseStamped {
  int64_t timestamp_ns;  ///< Pose timestamp in nanoseconds
  Pose pose;             ///< Pose (transformation between two coordinate frames)
};

/**
 * @brief Pose with covariance
 *
 * Pose covariance is defined via matrix exponential:
 * for a random zero-mean perturbation `u` in the tangent space
 * random pose is determined by `mean_pose * exp(u)`.
 */
struct PoseWithCovariance {
  Pose pose;                 ///< Pose (transformation between two coordinate frames)
  PoseCovariance covariance; /**< Row-major representation of the 6x6 covariance matrix.
                              The orientation parameters use a fixed-axis representation.
                              In order, the parameters are:
                              (rotation about X axis, rotation about Y axis, rotation about Z axis, x, y, z)
                              Rotation in radians, translation in meters.*/
};

/**
 * @brief Rig pose estimate from the tracker
 *
 * The rig coordinate frame is user-defined and depends on the extrinsic parameters of the cameras.
 * The cameras' coordinate frames may not match the rig coordinate frame - depending on camers extrinsics.
 * The world coordinate frame is an arbitrary 3D coordinate frame. It coincides with the rig coordinate frame at the
 * first frame.
 */
struct PoseEstimate {
  int64_t timestamp_ns;                              ///< Pose timestamp (in nanoseconds) will match image timestamp
  std::optional<PoseWithCovariance> world_from_rig;  ///< Transform from rig coordinate frame to world coordinate frame
};

/**
 * @brief Observation
 *
 * 2D point with coordinates in image.
 * (0, 0) is the top-left corner.
 */
struct Observation {
  uint64_t id;            ///< observation id
  float u;                ///< 0 <= u < image width; (0, 0) is the top-left corner
  float v;                ///< 0 <= v < image height; (0, 0) is the top-left corner
  uint32_t camera_index;  ///< camera index
};

/**
 * @brief Landmark
 *
 * 3D point with coordinates in meters in world frame
 */
struct Landmark {
  uint64_t id;      ///< landmark id
  Vector3f coords;  ///< x, y, z in meters in world frame
};

/**
 * @brief Visual Inertial Odometry (VIO) Tracker
 */
class CUVSLAM_API Odometry {
public:
  /// Image set
  using ImageSet = std::vector<Image>;
  /// Gravity vector
  using Gravity = Vector3f;

  /**
   * @brief Multicamera mode
   *
   * Multicamera mode defines which cameras will be used for mono SOF (primary cameras)
   */
  enum class MulticameraMode : uint8_t {
    /// primary cameras auto selection, each secondary camera must be connected to only one primary camera
    Performance,
    /// all cameras are primary cameras
    Precision,
    /// primary cameras auto selection, secondary cameras may be connected to more than one primary camera
    Moderate,
  };

  /**
   * @brief Odometry mode
   *
   * Odometry mode defines what data is expected and how it will be used by odometry tracker
   */
  enum class OdometryMode : uint8_t {
    Multicamera,  ///< Uses multiple synchronized stereo cameras, all cameras need to have frustum overlap with at least
                  ///< one another camera.
    Inertial,     ///< Uses stereo camera and IMU measurements. A single stereo-camera with a single IMU sensor is
                  ///< supported.
    RGBD,  ///< Uses RGB-D camera for tracking. A single RGB-D camera is supported. RGB & Depth images must be aligned.
    Mono   ///< Uses a single camera, tracking is accurate up to scale.
  };

  /**
   * @brief RGBD odometry settings
   */
  struct RGBDSettings {
    /// @brief Scale of provided depth measurements. Default: 1.f
    ///
    /// The scale factor is the denominator used to convert raw depth values from the input depth image to actual
    /// distance measurements in meters. For example, in [TUM
    /// RGB-D](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/file_formats#intrinsic_camera_calibration_of_the_kinect)
    /// a factor of 5000 is used for 16-bit PNG images, meaning each pixel value should be divided by 5000 to get the
    /// depth in meters, while a factor of 1 is used for 32-bit float images, where the depth values are already in
    /// meters.
    float depth_scale_factor = 1.f;

    /// @brief Depth camera id.
    ///
    /// Depth image is supposed to be pixel-to-pixel aligned with some RGB camera image.
    /// This field specifies camera id, that depth is aligned with. Default: -1
    int32_t depth_camera_id = -1;

    /// Allows stereo 2D tracking between depth-aligned camera and any other camera. Default: false
    bool enable_depth_stereo_tracking = false;
  };

  /**
   * @brief Configuration parameters of the VIO tracker
   */
  struct Config {
    /// Multicamera mode. Default: MulticameraMode::Precision
    MulticameraMode multicam_mode = MulticameraMode::Precision;
    /// Odometry mode. Default: OdometryMode::Multicamera
    OdometryMode odometry_mode = OdometryMode::Multicamera;
    /// Enable tracking using GPU. Default: true.
    bool use_gpu = true;
    /// Enable SBA asynchronous mode. Default: true.
    bool async_sba = true;
    /**
     * @brief Enable internal pose prediction mechanism. Default: true
     *
     * If frame rate is high enough it improves tracking performance and stability.
     * As a general rule it is better to use a pose prediction mechanism
     * tailored to a specific application. If you have an IMU, consider using
     * it to provide pose predictions to cuVSLAM.
     */
    bool use_motion_model = true;
    /// Enable image denoising. Disable if the input images have already passed through a denoising filter.
    /// Default: false
    bool use_denoising = false;
    /// Enable fast and robust tracking between rectified cameras with principal points on the horizontal line.
    /// Default: false
    bool rectified_stereo_camera = false;
    /// Enable GetLastObservations(). Warning: export flags slow down execution and result in additional memory usage.
    /// Default: false
    bool enable_observations_export = false;
    /// Enable GetLastLandmarks(). Warning: export flags slow down execution and result in additional memory usage.
    /// Default: false
    bool enable_landmarks_export = false;
    /// Enable GetFinalLandmarks(). Warning: export flags slow down execution and result in additional memory usage.
    /// This flag also sets enable_landmarks_export and enable_observations_export.
    /// Default: false
    bool enable_final_landmarks_export = false;
    /// Maximum frame delta in seconds. Odometry will warn if time delta between frames is higher than the threshold.
    /// Default: 1.f
    float max_frame_delta_s = 1.f;
    /// Directory where input data will be dumped in edex format.
    std::string_view debug_dump_directory;
    /// Enable IMU debug mode. Default: false
    bool debug_imu_mode = false;
    /// RGBD odometry settings.
    RGBDSettings rgbd_settings;
  };

  // TODO(vikuznetsov): remove when https://gcc.gnu.org/bugzilla/show_bug.cgi?id=88165 is fixed
  /// @brief Get default configuration.
  ///
  /// @see Config for default values.
  /// @return default configuration
  static Config GetDefaultConfig() { return Config{}; }

  /**
   * @brief State of the odometry tracker
   *
   * Only available if data export is enabled in Config.
   */
  struct State {
    struct Context;
    using ContextMap = std::unordered_map<uint8_t, std::shared_ptr<Context>>;

    uint64_t frame_id;               ///< Internal frame id
    int64_t timestamp_ns;            ///< Timestamp in nanoseconds
    Pose delta;                      ///< Pose change since last keyframe
    bool keyframe;                   ///< Is this frame a keyframe?
    bool warming_up;                 ///< Is the tracker in warming up phase?
    std::optional<Gravity> gravity;  ///< Optional gravity information. Only available in OdometryMode::Inertial mode.
    std::vector<Observation> observations;  ///< Observations for this frame
    std::vector<Landmark> landmarks;        ///< Landmarks for this frame
    ContextMap context;                     ///< Opaque context information for this frame (used internally by Slam)
  };

  /**
   * @brief Construct a tracker
   *
   * @param[in] rig  rig setup
   * @param[in] cfg  tracker configuration
   * @throws std::runtime_error if tracker fails to initialize
   * @throws std::invalid_argument if rig or config is invalid
   */
  Odometry(const Rig& rig, const Config& cfg = GetDefaultConfig());

  /**
   * @brief Move constructor
   *
   * @param[in] other other tracker
   */
  Odometry(Odometry&& other) noexcept;

  /// @brief Destructor
  ~Odometry();

  /**
   * @brief Track a rig pose using current frame
   *
   * Track current frame synchronously: the function blocks until the tracker has computed a pose.
   * By default, this function uses visual odometry to compute a pose.
   * If visual odometry tracker fails to compute a pose, in inertial mode the function returns the position
   * calculated from a user-provided IMU data.
   * If after several calls of Track() visual odometry is not able to recover,
   * then invalid pose will be returned.
   *
   * The track will output poses in the same coordinate system until a loss of tracking.
   *
   * Image timestamps have to match. cuVSLAM will use timestamp from the camera 0 image.
   * If a camera rig provides "almost synchronized" frames, the timestamps should be within 1 millisecond.
   * The number of images for tracker should not exceed rig->num_cameras.
   *
   * @param[in]  images  an array of synchronized images no more than rig->num_cameras.
   * Must use ImageData::DataType::UINT8. Partial ImageSet is supported, for example due to frame drops. Corresponding
   * cameras are identified by Image::camera_index.
   * @param[in]  masks  (Optional) an array of corresponding masks no more than rig->num_cameras.
   * Must use ImageData::DataType::UINT8. Partial ImageSet is supported, for example if mask is calculated on for some
   * cameras. Corresponding cameras are identified by Image::camera_index.
   * @param[in]  depths  (Optional) an array of corresponding depth images. One depth image is now supported.
   * Must use ImageData::Encoding::MONO and ImageData::DataType::UINT16 or ImageData::DataType::FLOAT32.
   *
   * @return On success `PoseEstimate` contains estimated rig pose, on failure `PoseEstimate::world_from_rig` will be
   * `nullopt`.
   * @throws std::invalid_argument if image parameters are invalid
   * @throws std::runtime_error in case of unexpected errors
   */
  PoseEstimate Track(const ImageSet& images, const ImageSet& masks = {}, const ImageSet& depths = {});

  /**
   * @brief Register IMU measurement
   *
   * If visual odometry loses camera position, it briefly continues execution
   * using user-provided IMU measurements while trying to recover the position.
   * You should call these functions in the order of ascending timestamps however many IMU measurements you have
   * between image acquisitions:
   *
   * - tracker.Track
   * - tracker.RegisterImuMeasurement
   * - ...
   * - tracker.RegisterImuMeasurement
   * - tracker.Track
   *
   * IMU measurement and frame image both have timestamps, so it is important to call these functions in
   * strict ascending order of timestamps. RegisterImuMeasurement is thread-safe so it's allowed to call
   * RegisterImuMeasurement and Track in parallel.
   *
   * @param[in] sensor_index Sensor index; must be 0, as only one sensor is supported now
   * @param[in] imu IMU measurements
   * @throws std::invalid_argument if IMU fusion is disabled or if called out of the order of timestamps
   * @see Track
   */
  void RegisterImuMeasurement(uint32_t sensor_index, const ImuMeasurement& imu);

  /**
   * @brief Get Last Observations
   *
   * Get an array of observations from the last VO frame for a specific camera
   *
   * @param[in] camera_index Index of the camera to get observations for
   * @return Array of observations
   * @throws std::invalid_argument if stats export is disabled
   * @see Observation
   */
  std::vector<Observation> GetLastObservations(uint32_t camera_index) const;

  /**
   * @brief Get Last Landmarks
   *
   * Get an array of landmarks from the last VO frame;
   * Landmarks are 3D points in the last camera frame.
   * @return Array of landmarks
   * @throws std::invalid_argument if stats export is disabled
   * @see Landmark
   */
  std::vector<Landmark> GetLastLandmarks() const;

  /**
   * @brief Get Last Gravity
   *
   * Get gravity vector in the last VO frame
   * @return Optional gravity vector. Empty if gravity is not yet available.
   * @throws std::invalid_argument if IMU fusion is disabled
   * @see Gravity
   */
  std::optional<Gravity> GetLastGravity() const;

  /**
   * @brief Get tracker state
   *
   * Only available if data export is enabled in Config.
   *
   * @param[out] state Odometry state to be filled
   * @throws std::invalid_argument if stats export is disabled
   * @see State
   */
  void GetState(Odometry::State& state) const;

  /**
   * @brief Get all final landmarks from all frames
   *
   * Landmarks are 3D points in the odometry start frame.
   * @return std::unordered_map<uint64_t, Vector3f>
   * @throws std::invalid_argument if stats export is disabled
   * @see Landmark
   */
  std::unordered_map<uint64_t, Vector3f> GetFinalLandmarks() const;

  /**
   * @brief Get primary camera indices used for tracking
   *
   * @return Vector of primary camera indices
   */
  const std::vector<uint8_t>& GetPrimaryCameras() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl;
};

/**
 * @brief Result type that can hold either success data or error information.
 *
 * For use in callbacks. Result::error_message should not outlive the callback scope.
 */
// TODO(C++23): replace with std::expected
template <typename T>
struct Result {
  std::optional<T> data;           ///< data
  std::string_view error_message;  ///< error message

  /// Create a success result
  /// @param[in] value data
  /// @return Result
  static Result<T> Success(T&& value) { return Result<T>{std::move(value), ""}; }

  /// Create an error result
  /// @param[in] message error message
  /// @return Result
  static Result<T> Error(std::string_view message) { return Result<T>{std::nullopt, message}; }
};

/**
 * Simultaneous Localization and Mapping (SLAM)
 */
class CUVSLAM_API Slam {
public:
  /// @brief Image set
  using ImageSet = std::vector<Image>;

  /**
   * @brief SLAM configuration parameters
   */
  struct Config {
    /// If empty, map is kept in memory only. Else, map is synced to disk (LMDB) at this path, allowing large-scale
    /// maps; if the path already exists it will be overwritten. To load an existing map, use LocalizeInMap(). To save
    /// map, use SaveMap().
    std::string_view map_cache_path = "";
    /// Enable GPU use for SLAM
    bool use_gpu = true;
    /// Synchronous mode (does not run a separate work thread if true)
    bool sync_mode = false;
    /// Enable reading internal data from SLAM (Pose Graph, Loop Closures, Landmarks, etc.).
    /// Additionally separate data layers are enabled by `EnableReadingData`.
    bool enable_reading_internals = false;
    /// Planar constraints. SLAM poses will be modified so that the camera moves on a horizontal plane.
    bool planar_constraints = false;
    /// Special SLAM mode for visual map building in case ground truth is present.
    /// Not realtime, no loop closure, no map global optimization, SBA must be in main thread.
    bool gt_align_mode = false;
    /// Size of map cell. Default is 0 (the size will be calculated from the camera baseline).
    float map_cell_size = 0.0f;
    /// Maximum distance from camera to landmark for inclusion in map. Default is 100 meters.
    float max_landmarks_distance = 100.f;
    /// Maximum number of poses in SLAM pose graph. 300 is suitable for real-time mapping.
    /// The special value 0 means unlimited pose-graph.
    uint32_t max_map_size = 300;
    /// Minimum time interval between loop closure events in milliseconds.
    /// 1000 is suitable for real-time mapping.
    uint32_t throttling_time_ms = 0;
  };

  // TODO(vikuznetsov): remove when https://gcc.gnu.org/bugzilla/show_bug.cgi?id=88165 is fixed
  /// Get default configuration
  /// @return default configuration
  static Config GetDefaultConfig() { return Config{}; }

  /**
   * @brief Localization settings for use in LocalizeInMap
   */
  struct LocalizationSettings {
    float horizontal_search_radius;  ///< horizontal search radius in meters
    float vertical_search_radius;    ///< vertical search radius in meters
    float horizontal_step;           ///< horizontal step in meters
    float vertical_step;             ///< vertical step in meters
    float angular_step_rads;         ///< angular step around vertical axis in radians
    bool enable_reading_internals;   ///< enable reading internal data from SLAM
  };

  /**
   * @brief Metrics
   */
  struct Metrics {
    int64_t timestamp_ns;                  ///< timestamp of these measurements (in nanoseconds)
    bool lc_status;                        ///< loop closure status
    bool pgo_status;                       ///< pose graph optimization status
    uint32_t lc_selected_landmarks_count;  ///< Count of Landmarks Selected
    uint32_t lc_tracked_landmarks_count;   ///< Count of Landmarks Tracked
    uint32_t lc_pnp_landmarks_count;       ///< Count of Landmarks in PNP
    uint32_t lc_good_landmarks_count;      ///< Count of Landmarks in LC
  };

  /**
   * @brief Data layer for SLAM
   */
  enum class DataLayer : uint8_t {
    Landmarks,             ///< Landmarks that are visible in the current frame
    Map,                   ///< Landmarks of the map
    LoopClosure,           ///< Map's landmarks that are visible in the last loop closure event
    PoseGraph,             ///< Pose Graph
    LocalizerProbes,       ///< Localizer probes
    LocalizerMap,          ///< Landmarks of the Localizer map (opened database)
    LocalizerLandmarks,    ///< Landmarks that are visible in the localization
    LocalizerLoopClosure,  ///< Landmarks that are visible in the final loop closure of the localization
    Max,
  };

  /**
   * @brief Pose graph node
   */
  struct PoseGraphNode {
    uint64_t id;     ///< node identifier
    Pose node_pose;  ///< node pose
  };

  /**
   * @brief Pose graph edge
   */
  struct PoseGraphEdge {
    uint64_t node_from;         ///< node id
    uint64_t node_to;           ///< node id
    Pose transform;             ///< transform
    PoseCovariance covariance;  ///< covariance
  };

  /**
   * @brief Pose graph
   */
  struct PoseGraph {
    int64_t timestamp_ns;              ///< timestamp of the pose graph in nanoseconds
    std::vector<PoseGraphNode> nodes;  ///< nodes list
    std::vector<PoseGraphEdge> edges;  ///< edges list
  };

  /**
   * @brief Landmark with additional information
   */
  struct Landmark {
    uint64_t id;      ///< identifier
    float weight;     ///< weight (ignored now)
    Vector3f coords;  ///< x, y, z in meters in world frame
  };

  /**
   * Landmarks array
   */
  struct Landmarks {
    int64_t timestamp_ns;  ///< timestamp of landmarks in nanoseconds; corresponds to the timestamp of the frame where
                           ///< the landmarks were observed
    std::vector<Landmark> landmarks;  ///< landmarks list
  };

  /**
   * @brief Localizer probe
   *
   * Debug data from localizer for internal use.
   */
  struct LocalizerProbe {
    uint64_t id;                ///< probe identifier
    Pose guess_pose;            ///< input hint
    Pose exact_result_pose;     ///< exact pose if localizer success
    float weight;               ///< input weight
    float exact_result_weight;  ///< result weight
    bool solved;                ///< true for solved, false for unsolved
  };

  /**
   * @brief Localizer probes array.
   *
   * Debug data from localizer for internal use.
   */
  struct LocalizerProbes {
    int64_t timestamp_ns;                ///< timestamp of localizer try in nanoseconds
    float size;                          ///< size of search area in meters
    std::vector<LocalizerProbe> probes;  ///< list of probes
  };

  /**
   * Construct a SLAM instance with rig and primary cameras
   * @param[in] rig Camera rig configuration
   * @param[in] primary_cameras Vector of primary camera indices
   * @param[in] config SLAM configuration
   * @throws std::runtime_error if SLAM initialization fails
   */
  Slam(const Rig& rig, const std::vector<uint8_t>& primary_cameras, const Config& config = GetDefaultConfig());

  /**
   * Move constructor
   * @param[in] other other SLAM instance
   */
  Slam(Slam&& other) noexcept;

  /// Destructor
  ~Slam();

  /**
   * Process tracking results from `Odometry::Track`. This should be called after each successful tracking.
   * @param[in] state Odometry state containing all tracking data
   * @param[in] gt_pose Optional ground truth pose. Should be provided if `gt_align_mode` is enabled, otherwise
   * should be nullptr.
   * @see `Odometry::Track`
   * @throws std::invalid_argument if `gt_pose` is passed incorrectly
   * @return On success `Pose` contains rig pose estimated by SLAM
   */
  Pose Track(const Odometry::State& state, const Pose* gt_pose = nullptr);

  /**
   * Set rig pose estimated by a user.
   * @param[in] pose rig pose estimated by a user
   */
  void SetSlamPose(const Pose& pose);

  /**
   * Get all SLAM poses for each frame.
   * @param[in] max_poses_count maximum number of poses to return
   * @param[out] poses Vector of poses with timestamps
   * This call could be blocked by slam thread.
   */
  void GetAllSlamPoses(std::vector<PoseStamped>& poses, uint32_t max_poses_count = 0) const;

  /**
   * Save SLAM database (map) to folder asynchronously.
   * This folder will be created, if it does not exist.
   * Contents of the folder will be overwritten.
   * @param[in] folder_name Folder name, where SLAM database (map) will be saved
   * @param[in] callback Callback function to be called when save is complete, may be called in a separate thread
   */
  void SaveMap(const std::string_view& folder_name, std::function<void(bool success)> callback) const;

  /// Localization callback, may be called in a separate thread
  using LocalizationCallback = std::function<void(const Result<Pose>& result)>;

  /**
   * Localize in the existing database (map) asynchronously.
   * Finds the position of the camera in existing SLAM database (map).
   * If successful, moves the SLAM pose to the found position.
   * @param[in] folder_name Folder name, which stores saved SLAM database (map)
   * @param[in] guess_pose Pointer to the proposed pose, where the robot might be
   * @param[in] images Observed images. Will be used if Config::slam_sync_mode = true
   * @param[in] settings Localization settings
   * @param[in] callback Callback function to be called when localization is complete, may be called in a separate
   * thread.
   * @note Errors will be reported in the callback.
   */
  void LocalizeInMap(const std::string_view& folder_name, const Pose& guess_pose, const ImageSet& images,
                     LocalizationSettings settings, LocalizationCallback callback);

  /**
   * Get SLAM metrics.
   * @param[out] metrics SLAM metrics
   */
  void GetSlamMetrics(Metrics& metrics) const;

  /**
   * Get list of last 10 loop closure poses with timestamps.
   * @param[out] poses Vector of poses with timestamps
   */
  void GetLoopClosurePoses(std::vector<PoseStamped>& poses) const;

  /**
   * Enable reading data layer.
   * @param[in] layer Data layer to enable/disable
   * @param[in] max_items_count Maximum number of items to allocate in the layer
   */
  void EnableReadingData(DataLayer layer, uint32_t max_items_count);

  /**
   * Disable reading data layer.
   * @param[in] layer Data layer to disable
   */
  void DisableReadingData(DataLayer layer);

  /**
   * Read landmarks from a given data layer. Enabled by `EnableReadingData`.
   * @param[in] layer Data layer to read
   * @return Landmarks
   */
  std::shared_ptr<const Landmarks> ReadLandmarks(DataLayer layer);

  /**
   * Read pose graph. Enabled by `EnableReadingData(DataLayer::PoseGraph)`.
   * @return Pose graph
   */
  std::shared_ptr<const PoseGraph> ReadPoseGraph();

  /**
   * Read localizer probes. Enabled by `EnableReadingData(DataLayer::LocalizerProbes)`.
   *
   * Debug data from localizer for internal use.
   * @return Localizer probes
   */
  std::shared_ptr<const LocalizerProbes> ReadLocalizerProbes();

  /**
   * Merge existing maps into one map.
   * @param[in] rig Camera rig configuration
   * @param[in] databases Input array of directories with existing databases
   * @param[in] output_folder Directory to save output database
   * @throws std::runtime_error if merge fails
   * @see Rig
   */
  static void MergeMaps(const Rig& rig, const std::vector<std::string_view>& databases,
                        const std::string_view& output_folder);

private:
  class Impl;
  std::unique_ptr<Impl> impl;
};

}  // namespace cuvslam
