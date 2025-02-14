# rgbd-camera-interface

## Description

A ROS package for interfacing with RGBD cameras, providing standardized access to depth and RGB data streams.

## Prerequisites

- Ubuntu 20.04 or later
- ROS Noetic or later
- OpenCV 4.2+
- PCL 1.10+

## Installation

1. Clone the repository:

```sh
cd ~/catkin_ws/src
git clone https://github.com/yourusername/rgbd-camera-interface.git
```

2. Install dependencies:

```sh
sudo apt-get update
sudo apt-get install ros-noetic-cv-bridge ros-noetic-pcl-ros
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the package:

```sh
cd ~/catkin_ws
catkin_make
```

## Configuration

Edit the config file in `config/camera_params.yaml` to set your camera parameters:

```yaml
camera:
  frame_rate: 30
  resolution: [640, 480]
  depth_scale: 0.001
```

## Usage

1. Source your workspace:

```sh
source ~/catkin_ws/devel/setup.bash
```

2. Launch the camera interface:

```sh
roslaunch rgbd_camera_interface camera.launch
```

## Topics

- `/camera/rgb/image_raw` - RGB image stream
- `/camera/depth/image_raw` - Depth image stream
- `/camera/points` - Point cloud data

## License

MIT License

## Contributing

Pull requests are welcome. For major changes, please open an issue first.
