#!/usr/bin/env python3
# coding: utf-8

from re import X
import pcl
#import pcl_helper

import numpy as np
import open3d as o3d
import struct
import ctypes
import rospy
import rospkg
import copy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.msg import Imu
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from random import randint
import math


def ros_to_pcl(ros_cloud):
    points_list = []
    color_list = []

    for data in pc2.read_points(ros_cloud, skip_nans=True):
        points_list.append([data[0], data[1], data[2], data[3]])
        # print ('x: ' + str(data[0]) + ', y : ' + str(data[1]) + ', z : ' + str(data[2]) + ' , rgb :  : ' + str(data[3]))
    
    pcl_data = pcl.PointCloud_PointXYZRGB()
    pcl_data.from_list(points_list)

    return pcl_data

# Converts a pcl PointXYZRGB to a ROS PointCloud2 message

def pcl_to_ros(pcl_array):
    ros_msg = PointCloud2()

    ros_msg.header.stamp = rospy.Time.now()
    
    ros_msg.header.frame_id = 'map'

    ros_msg.height = 1
    ros_msg.width = pcl_array.size

    ros_msg.fields.append(PointField(
                            name="x",
                            offset=0,
                            datatype=PointField.FLOAT32, count=1))
    ros_msg.fields.append(PointField(
                            name="y",
                            offset=4,
                            datatype=PointField.FLOAT32, count=1))
    ros_msg.fields.append(PointField(
                            name="z",
                            offset=8,
                            datatype=PointField.FLOAT32, count=1))
    ros_msg.fields.append(PointField(
                            name="rgb",
                            offset=16,
                            datatype=PointField.FLOAT32, count=1))

    ros_msg.is_bigendian = False
    ros_msg.point_step = 32
    ros_msg.row_step = ros_msg.point_step * ros_msg.width * ros_msg.height
    ros_msg.is_dense = False
    buffer = []

    for data in pcl_array:
        s = struct.pack('>f', data[3])
        i = struct.unpack('>l', s)[0]
        pack = ctypes.c_uint32(i).value

        r = (pack & 0x00FF0000) >> 16
        g = (pack & 0x0000FF00) >> 8
        b = (pack & 0x000000FF)

        buffer.append(struct.pack('ffffBBBBIII', data[0], data[1], data[2], 1.0, b, g, r, 0, 0, 0, 0))
        # print ('x: ' + str(data[0]) + ', y : ' + str(data[1]) + ', z : ' + str(data[2]) + ' , rgb :  : ' + str(data[3]))
    
    # ros_msg.data = "".join([str(_) for _ in buffer])
    
    # ros_msg.data = "".join(map(str,buffer))
    # buffer = []
    # print ('x: ' + str(ros_msg.data[0]) + ', y : ' + str(ros_msg.data[1]) + ', z : ' + str(ros_msg.data[2]) + ' , rgb :  : ' + str(ros_msg.data[3]))
    ros_msg.data = "".join(buffer)

    return ros_msg



def callback(input_ros_msg):
    cloud = ros_to_pcl(input_ros_msg)

    # 실행 코드 부분 
    print(cloud)

    cloud_new = pcl_to_ros(cloud) #PCL을 ROS 메시지로 변경     
    pub.publish(cloud_new)

if __name__ == "__main__":
    rospy.init_node('test2', anonymous=True)
    rospy.Subscriber('/livox/lidar', PointCloud2, callback)

    pub = rospy.Publisher("/livox/lidar_new", PointCloud2, queue_size=1)

    rospy.spin()