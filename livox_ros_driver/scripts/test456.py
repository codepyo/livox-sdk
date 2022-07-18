#!/usr/bin/env python

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

theta_y_trans = 1.57
theta_x_trans = 0

def float_to_rgb(float_rgb):
    
    s = struct.pack('>f', float_rgb)
    i = struct.unpack('>l', s)[0]
    pack = ctypes.c_uint32(i).value
			
    r = (pack & 0x00FF0000) >> 16
    g = (pack & 0x0000FF00) >> 8
    b = (pack & 0x000000FF)
			
    color = [r,g,b]
			
    return color

# Converts a ROS PointCloud2 message to a pcl PointXYZRGB

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
    
    ros_msg.data = "".join(map(str,buffer))
    # buffer = []
    # print ('x: ' + str(ros_msg.data[0]) + ', y : ' + str(ros_msg.data[1]) + ', z : ' + str(ros_msg.data[2]) + ' , rgb :  : ' + str(ros_msg.data[3]))
    # ros_msg.data = "".join(buffer)

    return ros_msg

# Converts an RGB list to the packed float format used by PCL
def rgb_to_float(color):
    hex_r = (0xff & color[0]) << 16
    hex_g = (0xff & color[1]) << 8
    hex_b = (0xff & color[2])

    hex_rgb = hex_r | hex_g | hex_b

    float_rgb = struct.unpack('f', struct.pack('i', hex_rgb))[0]

    return float_rgb

# Generates a random color
def random_color_gen():
    r = randint(0, 255)
    g = randint(0, 255)
    b = randint(0, 255)
    return [r, g, b]

# Converts a PCL XYZRGB point cloud to an XYZ point cloud (removes color info)
def XYZRGB_to_XYZ(XYZRGB_cloud):
    XYZ_cloud = pcl.PointCloud()
    points_list = []

    for data in XYZRGB_cloud:
        points_list.append([data[0], data[1], data[2]])

    XYZ_cloud.from_list(points_list)
    return XYZ_cloud

# Returns a list of randomized colors
def get_color_list(cluster_count):
    if (cluster_count > len(get_color_list.color_list)):
        for i in xrange(len(get_color_list.color_list), cluster_count):
            get_color_list.color_list.append(random_color_gen())
    return get_color_list.color_list

#----------------------------------------------------------------------------------------------------------------------------------------


# liDAR data receiving class
class rpScanReceiver():
    
    def __init__(self):
        rospy.init_node('test2', anonymous=False)
        self.sub = rospy.Subscriber('/livox/lidar', PointCloud2, self.callback)
        self.pub = rospy.Publisher('/lidar_new', PointCloud2, queue_size=10)
        
        # self.rate = rospy.Rate(10)
        
        # rospack = rospkg.RosPack()
        # pkg_path = rospack.get_path('')
        # full_path = pkg_path + '/scripts/' + 'lidar.txt'
        # self.f = open(full_path, 'a')
        
        # rospy.on_shutdown(self.ros_shutdown)
        
        # while not rospy.is_shutdown():
        #     rosp
        rospy.spin()
    
    #downSampling function (Create a VoxelGrid filter object for a input point cloud)
    def do_voxel_grid_downssampling(self,pcl_data,leaf_size):
        vox = pcl_data.make_voxel_grid_filter()
        vox.set_leaf_size(leaf_size, leaf_size, leaf_size) # The bigger the leaf size the less information retained
        return  vox.filter()

    def callback(self, input_ros_msg):
        
        cloud = ros_to_pcl(input_ros_msg)
        cloud_new = pcl_to_ros(cloud)      
        # print(type(cloud_new))
        # while not rospy.is_shutdown():
        #     self.pub.publish(cloud_new)
        #     rospy.sleep(1.0)
        self.pub.publish(cloud_new)
        # while not rospy.is_shutdown():
        #     self.pub.publish(input_ros_msg)
            # rospy.spin()
            # self.rate.sleep()
    
    def ros_shutdown(self):
        self.f.close()

# def callback(input_ros_msg):
        
#     cloud = ros_to_pcl(input_ros_msg)
#     cloud_new = pcl_to_ros(cloud)      
#     # print(type(cloud_new))
#     # while not rospy.is_shutdown():
#     #     self.pub.publish(cloud_new)
#     #     rospy.sleep(1.0)
#     pub.publish(cloud_new)
    
# rospy.init_node('test2', anonymous=False)
# rospy.Subscriber('/livox/lidar', PointCloud2, callback)
# pub = rospy.Publisher('/lidar_new', PointCloud2, queue_size=15)
# rospy.spin()

#------------------------------main---------------------------------
if __name__=="__main__":
    # rospy.init_node('test2', anonymous=False)
    rp = rpScanReceiver()
    # try : 
    #     rp = rpScanReceiver()
    # except rospy.ROSException : 
    #     pass 
    # rospy.init_node('test2', anonymous=False)
    # rospy.Subscriber('/livox/lidar', PointCloud2, callback)
    # pub = rospy.Publisher('/livox/lidar_new', PointCloud2, queue_size=10)
    # rospy.spin()