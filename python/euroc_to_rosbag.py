"""Example: Save images as rosbag1."""

import logging
import re
# 配置日志格式
logging.basicConfig(level=logging.INFO,
                    format='[%(asctime)s.%(msecs)03d][%(levelname)s][%(filename)s:%(lineno)d] %(message)s',
                    datefmt='%m-%d %H:%M:%S',
                    force=True)

import argparse

# from __future__ import annotations

import os

import numpy as np

from typing_extensions import List, Tuple

from rosbags.rosbag1 import Writer
from rosbags.typesys import Stores, get_typestore
from rosbags.typesys.stores.ros1_noetic import (
    builtin_interfaces__msg__Time as Time,
    sensor_msgs__msg__CompressedImage as CompressedImage,
    std_msgs__msg__Header as Header,
    geometry_msgs__msg__Quaternion as QuaternionMsg,
    geometry_msgs__msg__Vector3 as Vector3Msg,
    sensor_msgs__msg__Imu as Imu,
)

#################################################### 全局变量 ####################################################

IMG_TOPIC = '/gopro/img_raw'
IMU_TOPIC = '/gopro/imu'
FRAME_ID = 'gopro_link'


#################################################### 函数定义 ####################################################

def read_img_list(list_file: str) -> List[int]:
    """
    读取图像时间戳列表
    Args:
        list_file (str): 图像时间戳列表文件路径,格式为 CSV,每行包含一个时间戳,注释行以 # 开头
    Returns:
        (List[int]): 图像时间戳列表,单位为纳秒
    """

    ts_list = []
    with open(list_file, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue

            parts = line.split(',')
            if len(parts) != 2:
                continue

            ts_str = parts[0].strip()
            try:
                ts = int(ts_str)
            except ValueError:
                continue

            ts_list.append(ts)
        # end for
    # end with
    return ts_list
# end def read_img_list


def read_imu_list(list_file: str) -> List[Tuple[int, np.ndarray]]:
    """
    读取 IMU 数据列表
    Args:
        list_file (str): IMU 数据列表文件路径,格式为 CSV,每行包含一个时间戳和六个 IMU 测量值( wx, wy, wz, ax, ay, az ),注释行以 # 开头
    Returns:
        (List[Tuple[int, np.ndarray]]): IMU 数据列表,每个元素为一个元组,包含时间戳( 单位为纳秒 )和一个包含六个 IMU 测量值的 numpy 数组
    """
    imu_list: List[Tuple[int, np.ndarray]] = []
    with open(list_file, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue

            parts = line.split(',')
            if len(parts) != 7:
                continue

            ts_str = parts[0].strip()
            try:
                ts = int(ts_str)
                vals = [float(x) for x in parts[1:]]
            except ValueError:
                continue

            imu_list.append((ts, np.array(vals, dtype=np.float64)))
        # end for
    # end with
    return imu_list
# end def read_imu_list


def save_to_rosbag(img_topic: str,
                   imu_topic: str,
                   frame_id: str,
                   bag_path: str,
                   img_data_dir: str = None,
                   img_ts_list: List[int] = None,
                   imu_list: List[Tuple[int, np.ndarray]] = None,
                   img_interval: int = 1) -> None:
    """
    将图像和 IMU 数据保存到 ROS bag 文件
    Args:
        img_topic (str): 图像话题名称
        imu_topic (str): IMU 话题名称
        frame_id (str): 坐标系 ID
        bag_path (str): ROS bag 文件路径
        img_data_dir (str): 图像数据目录
        img_ts_list (List[int]): 图像时间戳列表,单位为纳秒
        imu_list (List[Tuple[int, np.ndarray]]): IMU 数据列表,每个元素为一个元组,包含时间戳( 单位为纳秒 )和一个包含六个 IMU 测量值的 numpy 数组
    """

    if img_data_dir is None and imu_list is None:
        logging.error("No image or IMU data provided to save.")
        return
    # end if

    if img_data_dir is not None and (img_ts_list is None or len(img_ts_list) == 0):
        logging.error("Image data directory provided but no valid image timestamps found.")
        return
    # end if

    orientation = QuaternionMsg(x=0.0, y=0.0, z=0.0, w=1.0)
    # ROS Imu covariance fields require a fixed-length array of 9 elements.
    orientation_covariance = np.eye(3, dtype=np.float64).reshape(-1)
    angular_velocity_covariance = np.eye(3, dtype=np.float64).reshape(-1)
    linear_acceleration_covariance = np.eye(3, dtype=np.float64).reshape(-1)

    typestore = get_typestore(Stores.ROS1_NOETIC)
    with Writer(bag_path) as writer:

        # 保存图像数据
        if img_data_dir is not None:
            conn_img = writer.add_connection(img_topic, CompressedImage.__msgtype__, typestore=typestore)

            for idx, timestamp in enumerate(img_ts_list):
                if idx % img_interval != 0:
                    continue
                # end if

                path = os.path.join(img_data_dir, f'{timestamp}.png')
                msg = CompressedImage(
                    Header(
                        seq=idx,
                        stamp=Time(sec=int(timestamp // 10**9), nanosec=int(timestamp % 10**9)),
                        frame_id=frame_id,
                    ),
                    format='png',
                    data=np.fromfile(path, dtype=np.uint8),
                )

                writer.write(
                    conn_img,
                    timestamp,
                    typestore.serialize_ros1(msg, CompressedImage.__msgtype__),
                )
            # end for
        # end if

        # 保存 IMU 数据
        if imu_list is not None:
            conn_imu = writer.add_connection(imu_topic, Imu.__msgtype__, typestore=typestore)

            for idx, (timestamp, vals) in enumerate(imu_list):
                # vals: [wx, wy, wz, ax, ay, az]
                msg = Imu(
                    header=Header(
                        seq=idx,
                        stamp=Time(sec=int(timestamp // 10**9), nanosec=int(timestamp % 10**9)),
                        frame_id=frame_id,
                    ),
                    orientation=orientation,
                    orientation_covariance=orientation_covariance,
                    angular_velocity=Vector3Msg(x=float(vals[0]), y=float(vals[1]), z=float(vals[2])),
                    angular_velocity_covariance=angular_velocity_covariance,
                    linear_acceleration=Vector3Msg(x=float(vals[3]), y=float(vals[4]), z=float(vals[5])),
                    linear_acceleration_covariance=linear_acceleration_covariance,
                )

                writer.write(
                    conn_imu,
                    timestamp,
                    typestore.serialize_ros1(msg, Imu.__msgtype__),
                )
            # end for
        # end if
    # end with

    logging.info(f"Data saved to ROS bag: {bag_path}")
# end def save_to_rosbag

#################################################### 主函数 ####################################################


if __name__ == '__main__':

    parser = argparse.ArgumentParser()

    parser.add_argument('--data_dir', type=str, required=True, help='Path to the Euroc dataset directory')
    parser.add_argument('--bag_path', type=str, required=True, help='Path to the output ROS bag file')
    parser.add_argument('--img_interval', type=int, default=1, help='Interval for saving images to ROS bag (default: 1, save all images)')

    args = parser.parse_args()

    data_dir = args.data_dir
    bag_path = args.bag_path
    img_interval = args.img_interval
    logging.info(f"Data directory: {data_dir}")
    logging.info(f"Output bag path: {bag_path}")
    logging.info(f"Image save interval: {img_interval}")
    print()

    img_ts_list = None
    img_data_dir = None

    img_list_file = os.path.join(data_dir, 'cam0/data.csv')
    img_ts_list = read_img_list(img_list_file)
    if not img_ts_list:
        logging.warning("No valid image timestamps found.")
    else:
        logging.info(f"Found {len(img_ts_list)} image timestamps.")

        img_data_dir = os.path.join(data_dir, 'cam0/data')

        if not os.path.isdir(img_data_dir):
            logging.error(f"Image data directory not found: {img_data_dir}")
            exit(1)
        # end if
    # end if

    imu_list_file = os.path.join(data_dir, 'imu0/data.csv')
    imu_list = read_imu_list(imu_list_file)
    if not imu_list:
        logging.warning("No valid IMU data found.")
        imu_list = None
    else:
        logging.info(f"Found {len(imu_list)} IMU measurements.")
    # end if

    if img_data_dir is None and imu_list is None:
        logging.error("No valid image or IMU data found to save.")
        exit(1)
    # end if

    save_to_rosbag(IMG_TOPIC, IMU_TOPIC, FRAME_ID,
                   bag_path,
                   img_data_dir, img_ts_list, imu_list, img_interval)

# end if __name__ == '__main__'
