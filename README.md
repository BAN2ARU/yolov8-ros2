# yolov8_ros2

ROS2(foxy version) wrapper for [YOLOv8](https://github.com/ultralytics/ultralytics). This package enables you to perform object detection and segmentation.

## Installation

### Prerequisites
- Ubuntu 20.04
- [CUDA](https://developer.nvidia.com/cuda-downloads)
- [CUDNN](https://developer.nvidia.com/cudnn-downloads)
- [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

### Build the package

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/ban2aru/yolov8-ros2.git
cd ..
sudo apt update
pip install -r ./src/yolov8-ros2/requirements.txt
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source ./install/setup.bash
```

## Usage

### Start the YOLOv8 node

```bash
ros2 launch yolov8_launch yolov8_foxy.launch.py
```

#### Object detection demo

```bash
ros2 launch yolov8_launch yolov8_foxy.launch.py img_topic:=<camera_image_topic>
```

#### Segmentation demo

```bash
ros2 launch yolov8_launch yolov8_foxy.launch.py weight:=yolov8n-seg.pt img_topic:=<camera_image_topic> 
```

#### Parameters
- **weight**: YOLOv8 model weight. Default is `yolov8n.pt`
- **device**: device type (GPU/CUDA/CPU). Default is `cuda:0`
- **conf_threshold**: NMS confidence threshold. Default is `0.5`
- **conf_threshold**: NMS IoU threshold. Default is `0.7`
- **image_reliability**: image reliability QOS for the image topic. Option is 0:system default | 1:Reliable | 2:Best Effort. Default is `2`
- **img_topic**: camera topic of image. Default is `/camera/rgb/image_raw`
