#include "rgbd-camera-interface/rgbd-camera-interface.h"
#include <cv_bridge/cv_bridge.h>
#include <rgbd_camera_interface/CameraControl.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <string>

class RGBDCameraNode {
public:
  RGBDCameraNode(ros::NodeHandle &nh) : nh_(nh) {
    // Get parameters
    nh_.param("left_camera_id", left_camera_id_, 0);
    nh_.param("right_camera_id", right_camera_id_, 1);
    nh_.param("frame_id", frame_id_, std::string("camera_link"));

    // Initialize publishers
    left_pub_ = nh_.advertise<sensor_msgs::Image>("left/image_raw", 1);
    right_pub_ = nh_.advertise<sensor_msgs::Image>("right/image_raw", 1);
    depth_pub_ = nh_.advertise<sensor_msgs::Image>("depth/image_raw", 1);

    // Initialize camera
    camera_.initialize(left_camera_id_, right_camera_id_);

    // Load parameters from config
    loadParams();

    // Initialize services
    control_service_ = nh_.advertiseService(
        "camera_control", &RGBDCameraNode::handleCameraControl, this);

    // Initialize camera info publishers
    left_info_pub_ =
        nh_.advertise<sensor_msgs::CameraInfo>("left/camera_info", 1);
    right_info_pub_ =
        nh_.advertise<sensor_msgs::CameraInfo>("right/camera_info", 1);
  }

  void run() {
    ros::Rate rate(30); // 30 Hz
    while (ros::ok()) {
      if (camera_.grab()) {
        publishImages();
        publishCameraInfo();
      } else {
        ROS_ERROR("Failed to grab frames");
      }
      ros::spinOnce();
      rate.sleep();
    }
  }

private:
  void loadParams() {
    // Load parameters from camera_params.yaml
    // ...implementation...
  }

  bool
  handleCameraControl(rgbd_camera_interface::CameraControl::Request &req,
                      rgbd_camera_interface::CameraControl::Response &res) {
    // Handle camera control commands
    // ...implementation...
    return true;
  }

  void publishImages() {
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = frame_id_;

    // Convert and publish left image
    sensor_msgs::ImagePtr left_msg =
        cv_bridge::CvImage(header, "bgr8", camera_.getLeftImage()).toImageMsg();
    left_pub_.publish(left_msg);

    // Convert and publish right image
    sensor_msgs::ImagePtr right_msg =
        cv_bridge::CvImage(header, "bgr8", camera_.getRightImage())
            .toImageMsg();
    right_pub_.publish(right_msg);

    // Convert and publish depth image
    sensor_msgs::ImagePtr depth_msg =
        cv_bridge::CvImage(header, "16UC1", camera_.getDepthMap()).toImageMsg();
    depth_pub_.publish(depth_msg);
  }

  void publishCameraInfo() {
    // Publish camera calibration info
    // ...implementation...
  }

  ros::NodeHandle nh_;
  rgbd_camera::RGBDCamera camera_;
  ros::Publisher left_pub_, right_pub_, depth_pub_;
  ros::Publisher left_info_pub_, right_info_pub_;
  ros::ServiceServer control_service_;
  int left_camera_id_, right_camera_id_;
  std::string frame_id_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "rgbd_camera_node");
  ros::NodeHandle nh("~");

  RGBDCameraNode node(nh);
  node.run();

  return 0;
}
