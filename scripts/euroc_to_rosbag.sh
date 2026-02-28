#!/bin/bash

# 内存检查用的前置指令
# valgrind --tool=memcheck --leak-check=full --show-leak-kinds=all --log-file=valgrind.log 

script_dir=$(dirname "$(realpath "$0")")
echo "当前脚本所在的目录: $script_dir"
echo

############################################# 参数配置 ##############################################

img_interval=1  # 每隔多少帧保存一张图像，1表示保存所有图像

# 数据目录
data_dir="/home/user/Temp/TmpCode/gopro_parser/bin/demo"

# 输出bag文件路径
bag_path="/home/user/Temp/TmpCode/gopro_parser/bin/demo.bag"


#############################################  可执行部分  #############################################

python3 $script_dir/../python/euroc_to_rosbag.py \
    --img_interval $img_interval \
    --data_dir $data_dir \
    --bag_path $bag_path 


echo
source /opt/ros/noetic/setup.bash

rosbag info $bag_path