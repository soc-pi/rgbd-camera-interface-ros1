#ifndef RGBD_CAMERA_INTERFACE_H
#define RGBD_CAMERA_INTERFACE_H

#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <string>

/**
 * @namespace rgbd_camera
 * @brief Namespace containing RGBD camera interface functionality
 */
namespace rgbd_camera {

/**
 * @class RGBDCamera
 * @brief Interface class for stereo RGBD camera operations
 *
 * This class provides a high-level interface for controlling and accessing
 * stereo camera pairs for RGBD (RGB + Depth) imaging applications.
 */
class RGBDCamera {
public:
  /**
   * @brief Default constructor
   */
  RGBDCamera();

  /**
   * @brief Destructor
   */
  ~RGBDCamera();

  /**
   * @brief Initialize the stereo camera system
   * @param left_camera_id ID of the left camera device
   * @param right_camera_id ID of the right camera device
   * @return true if initialization successful, false otherwise
   */
  bool initialize(int left_camera_id = 0, int right_camera_id = 1);

  /**
   * @brief Capture a frame from both cameras
   * @return true if frame capture successful, false otherwise
   */
  bool grab();

  /**
   * @brief Load camera calibration parameters from file
   * @param calib_file Path to the calibration file
   * @return true if calibration loaded successfully, false otherwise
   */
  bool loadCalibration(const std::string &calib_file);

  /**
   * @brief Set the resolution for both cameras
   * @param width Desired frame width in pixels
   * @param height Desired frame height in pixels
   * @return true if resolution set successfully, false otherwise
   */
  bool setResolution(int width, int height);

  /**
   * @brief Set the frame rate for both cameras
   * @param fps Desired frames per second
   * @return true if frame rate set successfully, false otherwise
   */
  bool setFrameRate(int fps);

  /**
   * @brief Set camera parameters for both cameras
   * @param exposure Exposure time in milliseconds
   * @param gain Camera gain value
   * @return true if parameters set successfully, false otherwise
   */
  bool setCameraParams(double exposure, double gain);

  /**
   * @brief Start the video streaming
   * @return true if streaming started successfully, false otherwise
   */
  bool startStreaming();

  /**
   * @brief Stop the video streaming
   * @return true if streaming stopped successfully, false otherwise
   */
  bool stopStreaming();

  /**
   * @brief Release camera resources
   */
  void release();

  // Getters
  /**
   * @brief Get the most recent left camera image
   * @return cv::Mat containing the left image
   */
  cv::Mat getLeftImage() const;

  /**
   * @brief Get the most recent right camera image
   * @return cv::Mat containing the right image
   */
  cv::Mat getRightImage() const;

  /**
   * @brief Get the computed depth map
   * @return cv::Mat containing the depth map
   */
  cv::Mat getDepthMap() const;

  /**
   * @brief Get the left camera matrix
   * @return cv::Mat containing the camera matrix
   */
  cv::Mat getLeftCameraMatrix() const;

  /**
   * @brief Get the right camera matrix
   * @return cv::Mat containing the camera matrix
   */
  cv::Mat getRightCameraMatrix() const;

  /**
   * @brief Get the left camera distortion coefficients
   * @return cv::Mat containing the distortion coefficients
   */
  cv::Mat getLeftDistCoeffs() const;

  /**
   * @brief Get the right camera distortion coefficients
   * @return cv::Mat containing the distortion coefficients
   */
  cv::Mat getRightDistCoeffs() const;

  /**
   * @brief Get the rotation matrix between the left and right cameras
   * @return cv::Mat containing the rotation matrix
   */
  cv::Mat getRotationMatrix() const;

  /**
   * @brief Get the translation vector between the left and right cameras
   * @return cv::Mat containing the translation vector
   */
  cv::Mat getTranslationVector() const;

  /**
   * @brief Check if the camera system is initialized
   * @return true if initialized, false otherwise
   */
  bool isInitialized() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace rgbd_camera

#endif // RGBD_CAMERA_INTERFACE_H
