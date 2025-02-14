#include "rgbd-camera-interface/rgbd-camera-interface.h"
#include <iostream>
#include <opencv2/highgui.hpp>

int main() {
  rgbd_camera::RGBDCamera camera;

  if (!camera.initialize()) {
    std::cerr << "Failed to initialize cameras!" << std::endl;
    return -1;
  }

  while (true) {
    if (!camera.grab()) {
      std::cerr << "Failed to grab frames!" << std::endl;
      break;
    }

    cv::imshow("Left Camera", camera.getLeftImage());
    cv::imshow("Right Camera", camera.getRightImage());
    cv::imshow("Depth Map", camera.getDepthMap());

    if (cv::waitKey(1) == 27) // ESC key
      break;
  }

  camera.release();
  return 0;
}
