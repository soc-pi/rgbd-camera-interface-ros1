#include "rgbd-camera-interface/rgbd-camera-interface.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

namespace rgbd_camera {

/**
 * @brief Implementation structure for RGBDCamera class
 *
 * This structure contains the actual implementation details of the RGBDCamera
 * class, following the PIMPL idiom.
 */
struct RGBDCamera::Impl {
  cv::VideoCapture left_cam_;
  cv::VideoCapture right_cam_;
  cv::Mat left_frame_;
  cv::Mat right_frame_;
  cv::Mat depth_map_;
  cv::Mat left_camera_matrix_;
  cv::Mat right_camera_matrix_;
  cv::Mat left_dist_coeffs_;
  cv::Mat right_dist_coeffs_;
  cv::Mat R_, T_;
  cv::Mat left_image_;
  cv::Mat right_image_;

  cv::Ptr<cv::StereoBM> stereo_matcher_;
  bool is_initialized_{false};
  bool is_calibrated_{false};
  bool is_streaming_{false};

  Impl() : stereo_matcher_(cv::StereoBM::create(16, 21)) {}
};

/**
 * @brief Constructs a new RGBDCamera object and initializes the implementation
 */
RGBDCamera::RGBDCamera() : impl_(new Impl()) {}

/**
 * @brief Destroys the RGBDCamera object and releases resources
 */
RGBDCamera::~RGBDCamera() = default;

/**
 * @brief Implements the camera initialization logic
 *
 * @details Opens the camera devices and prepares them for capturing.
 * Sets up initial parameters and verifies camera accessibility.
 */
bool RGBDCamera::initialize(int left_camera_id, int right_camera_id) {
  if (impl_->is_initialized_) {
    return false;
  }

  impl_->left_cam_.open(left_camera_id);
  impl_->right_cam_.open(right_camera_id);

  if (!impl_->left_cam_.isOpened() || !impl_->right_cam_.isOpened()) {
    release();
    return false;
  }

  impl_->is_initialized_ = true;
  return true;
}

/**
 * @brief Implements the calibration loading logic
 *
 * @details Reads camera calibration parameters from a YAML file and
 * sets up the stereo rectification matrices.
 */
bool RGBDCamera::loadCalibration(const std::string &calib_file) {
  if (!impl_->is_initialized_) {
    return false;
  }

  try {
    YAML::Node config = YAML::LoadFile(calib_file);

    // Load camera matrices
    auto leftMatrix = config["left_camera_matrix"];
    auto rightMatrix = config["right_camera_matrix"];
    impl_->left_camera_matrix_ = cv::Mat::zeros(3, 3, CV_64F);
    impl_->right_camera_matrix_ = cv::Mat::zeros(3, 3, CV_64F);

    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        impl_->left_camera_matrix_.at<double>(i, j) =
            leftMatrix[i * 3 + j].as<double>();
        impl_->right_camera_matrix_.at<double>(i, j) =
            rightMatrix[i * 3 + j].as<double>();
      }
    }

    // Load distortion coefficients
    auto leftDist = config["left_distortion"];
    auto rightDist = config["right_distortion"];
    impl_->left_dist_coeffs_ = cv::Mat(1, 5, CV_64F);
    impl_->right_dist_coeffs_ = cv::Mat(1, 5, CV_64F);

    for (int i = 0; i < 5; i++) {
      impl_->left_dist_coeffs_.at<double>(i) = leftDist[i].as<double>();
      impl_->right_dist_coeffs_.at<double>(i) = rightDist[i].as<double>();
    }

    // Load stereo parameters
    auto R = config["rotation_matrix"];
    auto T = config["translation_vector"];
    impl_->R_ = cv::Mat::zeros(3, 3, CV_64F);
    impl_->T_ = cv::Mat::zeros(3, 1, CV_64F);

    for (int i = 0; i < 3; i++) {
      impl_->T_.at<double>(i) = T[i].as<double>();
      for (int j = 0; j < 3; j++) {
        impl_->R_.at<double>(i, j) = R[i * 3 + j].as<double>();
      }
    }

    impl_->is_calibrated_ = true;
    return true;
  } catch (const std::exception &e) {
    impl_->is_calibrated_ = false;
    return false;
  }
}

bool RGBDCamera::setResolution(int width, int height) {
  if (!impl_->is_initialized_)
    return false;

  bool success = impl_->left_cam_.set(cv::CAP_PROP_FRAME_WIDTH, width) &&
                 impl_->left_cam_.set(cv::CAP_PROP_FRAME_HEIGHT, height) &&
                 impl_->right_cam_.set(cv::CAP_PROP_FRAME_WIDTH, width) &&
                 impl_->right_cam_.set(cv::CAP_PROP_FRAME_HEIGHT, height);

  return success;
}

bool RGBDCamera::setCameraParams(double exposure, double gain) {
  if (!impl_->is_initialized_)
    return false;

  bool success = impl_->left_cam_.set(cv::CAP_PROP_EXPOSURE, exposure) &&
                 impl_->right_cam_.set(cv::CAP_PROP_EXPOSURE, exposure) &&
                 impl_->left_cam_.set(cv::CAP_PROP_GAIN, gain) &&
                 impl_->right_cam_.set(cv::CAP_PROP_GAIN, gain);

  return success;
}

bool RGBDCamera::grab() {
  if (!impl_->is_initialized_ || !impl_->is_streaming_)
    return false;

  bool success = impl_->left_cam_.read(impl_->left_image_) &&
                 impl_->right_cam_.read(impl_->right_image_);

  if (success && impl_->is_calibrated_) {
    impl_->stereo_matcher_->compute(impl_->left_image_, impl_->right_image_,
                                    impl_->depth_map_);
  }

  return success;
}

cv::Mat RGBDCamera::getLeftImage() const { return impl_->left_image_; }

cv::Mat RGBDCamera::getRightImage() const { return impl_->right_image_; }

cv::Mat RGBDCamera::getDepthMap() const { return impl_->depth_map_; }

void RGBDCamera::release() {
  if (impl_->is_initialized_) {
    impl_->left_cam_.release();
    impl_->right_cam_.release();
    impl_->is_initialized_ = false;
    impl_->is_streaming_ = false;
  }
}

cv::Mat RGBDCamera::getLeftCameraMatrix() const {
  return impl_->left_camera_matrix_;
}

bool RGBDCamera::startStreaming() {
  if (!impl_->is_initialized_ || impl_->is_streaming_) {
    return false;
  }

  // Warm up cameras
  for (int i = 0; i < 5; i++) {
    impl_->left_cam_.grab();
    impl_->right_cam_.grab();
  }

  impl_->is_streaming_ = true;
  return true;
}

bool RGBDCamera::stopStreaming() {
  if (!impl_->is_streaming_) {
    return false;
  }

  impl_->is_streaming_ = false;
  return true;
}

bool RGBDCamera::setFrameRate(int fps) {
  if (!impl_->is_initialized_) {
    return false;
  }

  return impl_->left_cam_.set(cv::CAP_PROP_FPS, fps) &&
         impl_->right_cam_.set(cv::CAP_PROP_FPS, fps);
}

bool RGBDCamera::isInitialized() const { return impl_->is_initialized_; }

// ... Add other getter implementations ...

} // namespace rgbd_camera
