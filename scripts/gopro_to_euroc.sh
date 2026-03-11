#!/bin/bash

script_dir=$(dirname "$(realpath "$0")")
echo "当前脚本所在的目录: $script_dir"
echo

root_dir=$(dirname "$script_dir")
echo "项目根目录: $root_dir"
echo


############################################# 参数配置 ##############################################

# GoPro 视频文件路径
gopro_video_path="$root_dir/data/hero8.mp4"
# gopro_video_path="/home/user/Temp/TmpData/GoPro/GX011570.MP4"

# 数据保存目录,解析完成后会在该目录下生成:
# 1. cam0/data.csv 和 cam0/data 文件夹,分别保存图像时间戳列表和图像数据
# 2. imu0/data.csv 文件,分别保存 IMU 时间戳和数据
save_dir="$root_dir/bin/euroc/demo"
# save_dir="$root_dir/bin/euroc/room"

# 图像缩放比例, 1.0 表示不缩放, 0.5 表示缩小一半
scale=1.0

# 每隔多少帧保存一张图像,1表示保存所有图像
interval=1  

# 添加 --gray 表示是否保存灰度图像,默认值为 false
# 添加 --display 表示是否在处理过程中显示图像,默认值为 false
# 添加 --ignore_img 表示是否在处理过程中忽略图像数据,默认值为 false
# 添加 --ignore_imu 表示是否在处理过程中忽略 IMU 数据,默认值为 false


#############################################  可执行部分  #############################################

$script_dir/../bin/gopro_to_euroc \
    --g $gopro_video_path \
    --d $save_dir \
    --s $scale \
    --interval $interval \
    # --gray
    
