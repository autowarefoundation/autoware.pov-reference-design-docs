# ROS 2

## Dependencies

We can follow [the tutorial](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) to install ROS 2 humble.

```bash
# ROS2 packages
sudo apt install ros-humble-cv-bridge ros-humble-image-transport

# Build tools
sudo apt install cmake build-essential

# ONNX Runtime (Download from releases)
# TensorRT (NVIDIA SDK)
```

## Build

```bash
cd ROS2
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select sensors models visualization \
  --cmake-args \
  -DONNXRUNTIME_ROOTDIR=/path/to/onnxruntime \
  -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4 \
  -DCMAKE_BUILD_TYPE=Release
```

## Usage

```bash
source install/setup.bash
# SceneSeg
ros2 launch models run_pipeline.launch.py \
  pipeline:=scene_seg \
  video_path:="../data/video.mp4"
# SceneSeg
ros2 launch models run_pipeline.launch.py \
  pipeline:=domain_seg \
  video_path:="../data/video.mp4"
# SceneSeg
ros2 launch models run_pipeline.launch.py \
  pipeline:=scene3d \
  video_path:="../data/video.mp4"
```
