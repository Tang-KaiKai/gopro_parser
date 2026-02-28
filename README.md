# gopro_parser
This repository contains code for parsing GoPro telemetry metadata to obtain GoPro images with synchronized IMU measurements. The GoPro visual-inertial data can then be saved in **rosbag** or **EuRoC** format. Thus, effectively paving the way for **Visual-Inertial Odometry/SLAM** for GoPro cameras.    

This repository use [gpmf-parser](gpmf_parser/README.md) from [GoPro](https://github.com/gopro/gpmf-parser) to extract metadata and timing information from GoPro cameras.    

This repository references [gopro_ros](https://github.com/AutonomousFieldRoboticsLab/gopro_ros), and removed the ROS1 dependency

本仓库包含用于解析 GoPro 遥测元数据的代码,以获取带有同步 IMU 测量值的 GoPro 图像。GoPro 视觉惯性数据可以保存为 **rosbag** 或 **EuRoC** 格式。因此,这有效地为 GoPro 相机的**视觉惯性里程计/SLAM**铺平了道路。    

该存储库使用 [GoPro](https://github.com/gopro/gpmf-parser) 的 [gpmf-parser](gpmf_parser/README.md) 从 GoPro 相机中提取元数据和时间信息。

本仓库参考了[gopro_ros](https://github.com/AutonomousFieldRoboticsLab/gopro_ros),移除了 ROS1 依赖        


## 环境配置

### 依赖
- Ubuntu 20.04 or later
- [OpenCV](https://github.com/opencv/opencv) >= 3.2
- [FFmpeg](http://ffmpeg.org/)
- [rosbags](https://ternaris.gitlab.io/rosbags/)

### 安装

```bash
# OpenCV
sudo apt install libopencv-dev

# FFmpeg
sudo apt install ffmpeg

# rosbags
pip install rosbags
```

如果你的系统是 Ubuntu 22.04, 你还额外需要安装: 
```bash
sudo apt install libavdevice-dev libavfilter-dev libpostproc-dev libavcodec-dev libavformat-dev libswresample-dev
```

## 编译
```bash 
mkdir build
cd build
cmake ..
make -j32
```

## 使用

1. 导出为 EuRoC 格式
```
Usage: ./gopro_to_euroc
    --g             : the path to the GoPro video file
    --d             : the directory to save the extracted data
    --s             : the scale to resize the image, default: 1.0
    --gray          : whether to save grayscale images, default: false
    --display       : whether to display images during processing, default: false
    --ignore_img    : whether to ignore images during processing, default: false
    --ignore_imu    : whether to ignore IMU data during processing, default: false
    --h             : show this help message
```
可以参考[脚本](scripts/gopro_to_euroc.sh)


2. 将 EuRoC 格式的数据转为 rosbag
```
usage: euroc_to_rosbag.py [-h] --data_dir DATA_DIR --bag_path BAG_PATH [--img_interval IMG_INTERVAL]
```
可以参考[脚本](scripts/euroc_to_rosbag.sh)
