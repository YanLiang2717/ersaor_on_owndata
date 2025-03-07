
# coding=utf-8

import rospy
import rosbag
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
import pcl_ros  # 用于将 PCL 数据转换为 ROS PointCloud2
import pcl
import os

from pypcd import pypcd


def pcd_to_pointcloud2(pcd_file):
    
    
    pc = pypcd.PointCloud.from_path(pcd_file)
    
    # 提取XYZ数据并转换为列表
    x = pc.pc_data['x']
    y = pc.pc_data['y']
    z = pc.pc_data['z']
    points = np.stack([x, y, z], axis=1).tolist()
    
    # 创建PointCloud2消息头（时间戳后续覆盖）
    header = Header()
    header.frame_id = "base_link"
    
    # 生成PointCloud2消息（XYZ格式）
    cloud_msg = point_cloud2.create_cloud_xyz32(header, points)
    
    return cloud_msg

def convert_to_bag(tf_file, pcd_folder, output_bag):
    # 获取 PCD 文件列表并按文件名排序
    pcd_files = sorted([f for f in os.listdir(pcd_folder) if f.endswith(".pcd")])
    pcd_files = [os.path.join(pcd_folder, f) for f in pcd_files]  # 添加完整路径

    # 检查 PCD 文件数量是否与 TF 文件行数匹配
    with open(tf_file, 'r') as f:
        tf_lines = f.readlines()
    
    if len(pcd_files) != len(tf_lines):
        rospy.logerr("Number of PCD files ({}) does not match number of TF lines ({})".format(
            len(pcd_files), len(tf_lines)))
        return
    bag = rosbag.Bag(output_bag, 'w')

    for idx, line in enumerate(tf_lines):
        # 解析 TF 数据
        parts = line.strip().split()
        timestamp = float(parts[0])  # 时间戳
        x, y, z, qx, qy, qz, qw = map(float, parts[1:])
        time = rospy.Time.from_sec(timestamp)

        # 创建 TF 消息
        tf_msg = TransformStamped()
        tf_msg.header.stamp = time
        tf_msg.header.frame_id = "map"
        tf_msg.child_frame_id = "base_link"
        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = z
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw

        # 写入 TF 数据到 bag
        tf_bag_msg = TFMessage([tf_msg])
        bag.write("/tf", tf_bag_msg, time)

        # 获取对应的 PCD 文件
        pcd_file = pcd_files[idx]
        if os.path.exists(pcd_file):
            cloud_msg = pcd_to_pointcloud2(pcd_file)
            cloud_msg.header.stamp = time
            cloud_msg.header.frame_id = "base_link"
            bag.write("/rslidar_points", cloud_msg, time)
        else:
            rospy.logwarn("PCD file not found: {}".format(pcd_file))

    bag.close()

if __name__ == "__main__":
    rospy.init_node("data_to_bag")
    tf_file = "/home/l__e__i/ToHongQ_20250226/pose"  # 替换为你的 TF 文件路径
    pcd_folder = "/home/l__e__i/ToHongQ_20250226"  # 替换为你的 PCD 文件夹路径
    output_bag = "/home/l__e__i/ToHongQ_20250226/output.bag"  # 输出的 bag 文件路径
    convert_to_bag(tf_file, pcd_folder, output_bag)