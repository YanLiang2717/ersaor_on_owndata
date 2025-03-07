#!/usr/bin/env python2
# -*- coding: UTF-8 -*-

import rospy
import pcl2
import pcl_ros
import os
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header

def pcd_to_pointcloud2(pcd_file, frame_id="lidar"):
    """
    将 PCD 文件转换为 ROS 的 PointCloud2 消息
    :param pcd_file: PCD 文件路径
    :param frame_id: 点云的坐标系 ID
    :return: sensor_msgs/PointCloud2 消息
    """
    # 读取 PCD 文件
    if not os.path.exists(pcd_file):
        rospy.logerr("PCD 文件不存在: %s", pcd_file)
        return None

    cloud = pcl2.load(pcd_file)
    points = cloud.to_array()

    # 创建 PointCloud2 消息
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id

    cloud_msg = pcl_ros.point_cloud2.create_cloud_xyz32(header, points)
    return cloud_msg

def parse_tf_file(tf_file, parent_frame="world", child_frame="lidar"):
    """
    解析 TF 文件并返回 TFMessage
    :param tf_file: TF 文件路径
    :param parent_frame: 父坐标系 ID
    :param child_frame: 子坐标系 ID
    :return: tf2_msgs/TFMessage 消息
    """
    transforms = []
    if not os.path.exists(tf_file):
        rospy.logerr("TF 文件不存在: %s", tf_file)
        return None

    with open(tf_file, 'r') as f:
        for line in f:
            # 假设每行格式为：timestamp x y z qx qy qz qw
            parts = line.strip().split()
            if len(parts) != 8:
                rospy.logwarn("TF 文件格式错误: %s", line)
                continue

            timestamp = float(parts[0])
            x, y, z = map(float, parts[1:4])
            qx, qy, qz, qw = map(float, parts[4:8])

            # 创建 TransformStamped 消息
            tf_stamped = TransformStamped()
            tf_stamped.header.stamp = rospy.Time(timestamp)
            tf_stamped.header.frame_id = parent_frame
            tf_stamped.child_frame_id = child_frame
            tf_stamped.transform.translation.x = x
            tf_stamped.transform.translation.y = y
            tf_stamped.transform.translation.z = z
            tf_stamped.transform.rotation.x = qx
            tf_stamped.transform.rotation.y = qy
            tf_stamped.transform.rotation.z = qz
            tf_stamped.transform.rotation.w = qw

            transforms.append(tf_stamped)

    # 创建 TFMessage
    tf_msg = TFMessage(transforms)
    return tf_msg

def publish_data(pcd_dir, tf_dir, pcd_topic="/rslidar_points", tf_topic="/tf", rate=10):
    """
    发布 PCD 和 TF 数据到 ROS 话题
    :param pcd_dir: PCD 文件目录
    :param tf_dir: TF 文件目录
    :param pcd_topic: 点云话题名称
    :param tf_topic: TF 话题名称
    :param rate: 发布频率 (Hz)
    """
    rospy.init_node('data_publisher', anonymous=True)
    pcl_pub = rospy.Publisher(pcd_topic, PointCloud2, queue_size=10)
    tf_pub = rospy.Publisher(tf_topic, TFMessage, queue_size=10)

    # 获取 PCD 和 TF 文件列表
    pcd_files = sorted([os.path.join(pcd_dir, f) for f in os.listdir(pcd_dir) if f.endswith('.pcd')])
    #tf_files = sorted([os.path.join(tf_dir, f) for f in os.listdir(tf_dir) if f.endswith('.txt')])

    # 解析 TF 文件
    tf_msgs = parse_tf_file(tf_dir)
    if not tf_msgs:
        rospy.logerr("未能解析任何 TF 数据")
        return

    rate = rospy.Rate(rate)  # 发布频率
    for i, pcd_file in enumerate(pcd_files):
        if rospy.is_shutdown():
            break

        # 发布点云数据
        cloud_msg = pcd_to_pointcloud2(pcd_file)
        if cloud_msg:
            pcl_pub.publish(cloud_msg)
            rospy.loginfo("发布点云: %s", pcd_file)

        # 发布对应的 TF 数据
        if i < len(tf_msgs.transforms):
            tf_pub.publish(tf_msgs.transforms[i])  # 发布单个 TF 信息
            rospy.loginfo("发布 TF: %s", tf_msgs.transforms[i].header.stamp)  # 记录时间戳或其他信息

        rate.sleep()

if __name__ == '__main__':
    try:
        # 设置 PCD 和 TF 文件目录
        pcd_dir = "/home/l__e__i/lidar_motion/parse_data2/2024-12-19-10-40-06/iv_points"  # 替换为你的 PCD 文件目录
        tf_dir = "/home/l__e__i/lidar_motion/data/2024-12-19-10-40-06/poses.txt"    # 替换为你的 TF 文件目录

        # 发布数据
        publish_data(pcd_dir, tf_dir)
    except rospy.ROSInterruptException:
        pass
