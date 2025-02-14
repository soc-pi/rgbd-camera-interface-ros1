# rgbd-camera-interface

## Description

A ROS package for interfacing with RGBD cameras, providing standardized access to depth and RGB data streams.

## Prerequisites

- Docker (recommended)
- Ubuntu 20.04 or later (for non-Docker usage)
- ROS Noetic or later (for non-Docker usage)
- OpenCV 4.2+
- PCL 1.10+

## Installation & Build

### Using Docker (Recommended)

1. Clone the repository:

```sh
git clone https://github.com/yourusername/rgbd-camera-interface.git
cd rgbd-camera-interface
```

2. Build the Docker image:

```sh
make docker-build
```

3. Build the project:

```sh
make
```

### Without Docker

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

## Development

### Docker Commands

- Build the project: `make`
- Clean build files: `make clean`
- Rebuild everything: `make rebuild`
- Run tests: `make test`
- Get an interactive shell: `make docker-shell`
- Run the application: `make docker-run`

### Environment Variables

- `DOCKER_IMG`: Docker image name (default: noetic-ros1-dev)
- `DOCKER_TAG`: Docker image tag (default: latest)

## Configuration

Edit the config file in `config/camera_params.yaml` to set your camera parameters:

```yaml
camera:
  frame_rate: 30
  resolution: [640, 480]
  depth_scale: 0.001
```

## Usage

### Using Docker

1. Run the application:

```sh
make docker-run
```

### Without Docker

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
