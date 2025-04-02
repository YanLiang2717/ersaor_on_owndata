#!/usr/bin/env python2
# -*- coding: UTF-8 -*-
import rospy
import rosbag
import os
import tf
import numpy as np
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped, Transform, Pose
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pcl2
from sensor_msgs.msg import PointField
from erasor.msg import node

import argparse

def process_parameters():
    parser = argparse.ArgumentParser(description='Convert own data to erasor-type data')
    parser.add_argument('--lidar_topic', type=str, default="/rslidar_points", help='lidar_topic')
    parser.add_argument('--world_frame', type=str, default="map", help='world_frame')
    parser.add_argument('--lidar_frame', type=str, default="base_link", help='lidar_frame')
    
    args = parser.parse_args()
    return args

def filter_nan_points(point_cloud):
    # 创建一个新的点云消息
    filtered_points = PointCloud2()
    filtered_points.header = point_cloud.header
    filtered_points.height = point_cloud.height
    filtered_points.width = point_cloud.width

    # 解析点云数据
    points = list(pcl2.read_points(point_cloud, field_names=("x", "y", "z", "intensity"), skip_nans=True))

    # 将数据转换为numpy数组
    points_np = np.array(points, dtype=np.float32)

    # 将数据重新打包成PointCloud2格式
    filtered_points = pcl2.create_cloud_xyz32(filtered_points.header, points_np[:, :3])

    # 添加强度字段（如果有的话）
    if "intensity" in point_cloud.fields:
        intensity_field = next(f for f in point_cloud.fields if f.name == "intensity")
        intensity_values = points_np[:, intensity_field.offset].tolist()
        intensity_field_data = pcl2.create_cloud_field("float32", intensity_field.count, "intensity", intensity_values)
        filtered_points.fields.append(intensity_field_data)

    return filtered_points

class PointCloudSubscriber(object):

    def __init__(self,args):
        self.bag = rosbag.Bag(os.path.join("/home/l__e__i/2024-04-23-19-51-29/video0_data6751-9000jpg", "erasor_data.bag"),'w', compression=rosbag.Compression.NONE)
        self.listener = tf.TransformListener()
        self.lidar_topic = args.lidar_topic
        self.world_frame = args.world_frame
        self.lidar_frame = args.lidar_frame
        self.sub_lidar = rospy.Subscriber(self.lidar_topic, PointCloud2, self.lidar_callback, queue_size=5)
        self.sub_tf = rospy.Subscriber("/tf", TFMessage, self.tf_callback, queue_size=10)
        self.current_tf = None  # 存储最新的 TF 数据

        print("lidar_topic: {}".format(self.lidar_topic))
        print("world_frame: {}".format(self.world_frame))
        print("lidar_frame: {}".format(self.lidar_frame))

    def tf_callback(self, msg):
        # 保存最新的 TF 数据
        self.current_tf = msg

    def lidar_callback(self, msg):
        assert isinstance(msg, PointCloud2)

        if self.current_tf is None:
            rospy.logwarn("No TF data received yet.")
            return

        # 获取最新的 TF 数据
        tf_stamped = None
        for transform in self.current_tf.transforms:
            if transform.child_frame_id == self.lidar_frame and transform.header.frame_id == self.world_frame:
                tf_stamped = transform
                break

        if tf_stamped is None:
            rospy.logwarn("No TF data found for lidar_frame: {} and world_frame: {}".format(
                self.lidar_frame, self.world_frame))
            return

        # 写入 TF 数据到 bag
        self.bag.write('/tf', self.current_tf, tf_stamped.header.stamp)

        # 创建 Pose 消息
        pose_msg = Pose()
        pose_msg.position.x = tf_stamped.transform.translation.x
        pose_msg.position.y = tf_stamped.transform.translation.y
        pose_msg.position.z = tf_stamped.transform.translation.z
        pose_msg.orientation.x = tf_stamped.transform.rotation.x
        pose_msg.orientation.y = tf_stamped.transform.rotation.y
        pose_msg.orientation.z = tf_stamped.transform.rotation.z
        pose_msg.orientation.w = tf_stamped.transform.rotation.w

        # 创建自定义消息
        header = Header()
        header.frame_id = "base_link"
        header.stamp = tf_stamped.header.stamp
        out = node()
        out.header = header
        out.odom = pose_msg

        # 处理点云数据
        msg.header.frame_id = "base_link"
        msg.header.stamp = tf_stamped.header.stamp
        filtered_points = filter_nan_points(msg)
        out.lidar = filtered_points

        # 写入 bag 文件
        self.bag.write('/node/combined/optimized', out, out.header.stamp)
        self.bag.write('/debug/pc_raw', filtered_points, out.header.stamp)
        self.bag.write('/debug/pc_label', filtered_points, out.header.stamp)

        print(pose_msg)

    def __del__(self):
        print("## OVERVIEW ##")
        print(self.bag)
        self.bag.close()
        
if __name__ =='__main__':
    rospy.init_node("pointcloud_subscriber")
    args = process_parameters()
    PointCloudSubscriber(args)
    rospy.spin()