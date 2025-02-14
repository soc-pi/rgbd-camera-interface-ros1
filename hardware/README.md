# Waveshare Binocular Camera Module Dual IMX219

## Overview

The Waveshare Binocular Camera Module Dual IMX219 is a stereo vision camera module designed for depth perception, primarily compatible with NVIDIA Jetson boards like the Jetson Nano and Jetson Xavier NX. It features dual Sony IMX219 8-megapixel sensors, making it well-suited for applications requiring depth estimation, stereo vision, and 3D perception.

## Key Features

- **Dual IMX219 Sensors:** Each sensor provides 8 MP resolution (3280 × 2464)
- **Stereo Vision:** Enables depth perception for 3D imaging and computer vision tasks
- **MIPI CSI-2 Interface:** Directly connects to Jetson Nano/Xavier NX via the CSI camera interface
- **Wide Compatibility:** Primarily supports NVIDIA Jetson platforms but can also be adapted for Raspberry Pi
- **Fixed Baseline:** Designed for stereo imaging, with a fixed distance between the two cameras
- **Adjustable Lenses:** Allows tuning for focus and stereo calibration
- **High FPS Support:** Can achieve up to 30 FPS at 1080p and 60 FPS at lower resolutions

## Common Applications

- **Robotics:** Used for SLAM (Simultaneous Localization and Mapping) in autonomous robots and drones
- **Depth Estimation:** Generates depth maps by computing stereo disparities
- **Machine Vision:** Helps in industrial automation for object detection
- **3D Reconstruction:** Captures stereo images for 3D modeling
- **AI & Computer Vision:** Works with deep learning models for facial recognition, object tracking, and gesture detection

## Compatibility & Setup

- Best suited for Jetson Nano/Xavier NX, as NVIDIA provides stereo camera software support
- Requires stereo vision processing, which can be done using OpenCV or NVIDIA's VPI (Vision Programming Interface)
- Can be used with ROS (Robot Operating System) for robotic applications

> Need help with setup? Feel free to open an issue for specific use cases! 🚀
