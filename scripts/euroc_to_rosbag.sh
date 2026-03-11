#!/bin/bash

script_dir=$(dirname "$(realpath "$0")")
echo "当前脚本所在的目录: $script_dir"
echo

root_dir=$(dirname "$script_dir")
echo "项目根目录: $root_dir"
echo

############################################# 参数配置 ##############################################

# 每隔多少帧保存一张图像, 1表示保存所有图像
interval=1  

# 数据目录
data_dir="$root_dir/bin/euroc/demo"

# 输出bag文件路径
bag_path="$root_dir/bin/rosbag/demo.bag"


#############################################  可执行部分  #############################################

python3 $root_dir/python/euroc_to_rosbag.py \
    --interval $interval \
    --data_dir $data_dir \
    --bag_path $bag_path 


############################################# 结果展示  #############################################

echo
echo "使用 ROS1 的 rosbag 工具查看生成的 bag 文件信息: "
source /opt/ros/noetic/setup.bash

rosbag info $bag_path