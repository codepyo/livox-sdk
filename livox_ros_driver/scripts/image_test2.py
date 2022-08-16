#!/usr/bin/env python

from email.headerregistry import AddressHeader
from re import X
from tkinter import Y
import pcl
#import pcl_helper4

import numpy as np
import open3d as o3d
import struct
import ctypes
import rospy
import rospkg
import copy
import cv2
import matplotlib.image as mpimg
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Imu
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
from random import randint
import math
import ros_numpy
import sensor_msgs


# def sheet_array_to_pointcloud2(point, colors, stamp=None, frame_id=None):
def sheet_array_to_pointcloud2(cloud_arr, nx, ny, stamp=None, frame_id=None):
    
    point = np.asarray(cloud_arr.points)
    colors = np.asarray(cloud_arr.colors)
    points = []
    
    initx = 126.56 - 12.185 + 1.04 + 69 - 2.0 - 18.362
    inity = -69.301 - 3.1638 - 4.5118 + 1.0547 + 14.5 - 4.9245
    
    for i in range(556516):

        # x = point[i][0] * 30.0 * 1.05  + nx + initx
        # y = point[i][1] * 30.0 * 1.05  + ny + inity
        # z = point[i][2] * 30.0 * 1.05  
        
        x = point[i][0] * 0.8 * 100.0 + nx + initx
        y = point[i][1] * 0.8 * 100.0 + ny + inity
        z = point[i][2] * 0.8 * 100.0
        
        pt = [x,y,z,0]
        
        r = int(colors[i][0]*255.0)
        g = int(colors[i][1]*255.0)
        b = int(colors[i][2]*255.0)
        
        if(r>=200 and g>=200 and b>=200): continue
        
        r = 216
        g = 0
        b = 0
        
        a = 255
        rgb = struct.unpack('I', struct.pack('BBBB',b, g, r, a))[0]
        pt[3] = rgb
        points.append(pt)
    
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              PointField('rgb', 16, PointField.UINT32, 1)
            ]
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "output"
    pc2 = point_cloud2.create_cloud(header, fields, points)
    
    return pc2


def convert_imgTo_pcd(img, **kwargs):
    f = kwargs.get('f',None)

    l , w, c  = img.shape
    intensity_f_r = np.reshape(img[:,:,0],(l*w,1))
    intensity_f_g = np.reshape(img[:,:,1],(l*w,1))
    intensity_f_b = np.reshape(img[:,:,2],(l*w,1))

    # intensity_f_r = np.reshape(img[:,:,0],(l*w,1))
    # intensity_f_g = np.reshape(img[:,:,1],(l*w,1))
    # intensity_f_b = np.reshape(img[:,:,2],(l*w,1))
    x = np.arange(0,w,1)
    y = np.arange(l,0,-1)
    mesh_x, mesh_y = np.meshgrid(x,y)
    x_f = np.reshape(mesh_x,(l*l,1))
    y_f = np.reshape(mesh_y,(l*l,1))
    z_f = np.zeros((l*l,1))
    my_img_position = np.concatenate((x_f,y_f,z_f),axis=1)
    my_img_position = my_img_position/l

    # if distort == 1:
    #     my_img_position = distort_pcd(my_img_position)

    my_img_color = np.concatenate((intensity_f_r,intensity_f_g,intensity_f_b),axis=1)
    my_img_color = ((my_img_color - my_img_color.min()) / (my_img_color.max() - my_img_color.min()))
    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(my_img_position)
    pcd.colors = o3d.utility.Vector3dVector(my_img_color)
    
    mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
    R = mesh.get_rotation_matrix_from_xyz((np.pi, np.pi ,0))
    pcd.rotate(R, center = (0,0,10))
    # o3d.io.write_point_cloud("image_pcd.ply", pcd)
    # pcd_load = o3d.io.read_point_cloud("image_pcd.ply")
    # o3d.visualization.draw_geometries([pcd_load])
    # return my_img_position, my_img_color
    return pcd


class imageReceiver():
    
    def __init__(self):
        rospy.init_node('test2', anonymous=True)
        
        # self.sub = rospy.Subscriber('/livox/lidar', PointCloud2, self.callback)
        # self.sub = rospy.Subscriber('/livox/imu', Imu, self.callbackImu)
        
        
        # # self.image_pub = rospy.Publisher("/imagetopic2",Image)
        # # self.bridge = CvBridge()
        
        # # self.pub1 = rospy.Publisher('/marker1', Marker, queue_size=1)
        # # self.pub2 = rospy.Publisher('/marker2', Marker, queue_size=1)
        
        # self.pub = rospy.Publisher("/out", PointCloud2, queue_size=1)
        self.image_pub1 = rospy.Publisher("/picout1", PointCloud2, queue_size=1)
        self.image_pub2 = rospy.Publisher("/picout2", PointCloud2, queue_size=1)
        self.image_pub3 = rospy.Publisher("/picout3", PointCloud2, queue_size=1)
        self.image_pub4 = rospy.Publisher("/picout4", PointCloud2, queue_size=1)
        self.image_pub5 = rospy.Publisher("/picout5", PointCloud2, queue_size=1)
        self.image_pub6 = rospy.Publisher("/picout6", PointCloud2, queue_size=1)
        self.image_pub7 = rospy.Publisher("/picout7", PointCloud2, queue_size=1)
        self.image_pub8 = rospy.Publisher("/picout8", PointCloud2, queue_size=1)
        self.image_pub9 = rospy.Publisher("/picout9", PointCloud2, queue_size=1)
        
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            # img1 = mpimg.imread('parkpic61.jpg')
            # img1 = cv2.resize(img1,(256,256))
            # pcd_load1 = convert_imgTo_pcd(img1)
            # ros_cloud1 = sheet_array_to_pointcloud2(pcd_load1,0,0)
            # self.image_pub1.publish(ros_cloud1)

            # img2 = mpimg.imread('parkpic62.jpg')
            # img2 = cv2.resize(img2,(256,256))
            # pcd_load2 = convert_imgTo_pcd(img2)
            # ros_cloud2 = sheet_array_to_pointcloud2(pcd_load2,-45,0)
            # self.image_pub2.publish(ros_cloud2)

            # img3 = mpimg.imread('parkpic63.jpg')
            # img3 = cv2.resize(img3,(256,256))
            # pcd_load3 = convert_imgTo_pcd(img3)
            # ros_cloud3 = sheet_array_to_pointcloud2(pcd_load3,-90,0)
            # self.image_pub3.publish(ros_cloud3)

            # img4 = mpimg.imread('parkpic64.jpg')
            # img4 = cv2.resize(img4,(256,256))
            # pcd_load4 = convert_imgTo_pcd(img4)
            # ros_cloud4 = sheet_array_to_pointcloud2(pcd_load4,0,43)
            # self.image_pub4.publish(ros_cloud4)

            # img5 = mpimg.imread('parkpic65.jpg')
            # img5 = cv2.resize(img5,(256,256))
            # pcd_load5 = convert_imgTo_pcd(img5)
            # ros_cloud5 = sheet_array_to_pointcloud2(pcd_load5,-45,43)
            # self.image_pub5.publish(ros_cloud5)

            # img6 = mpimg.imread('parkpic66.jpg')
            # img6 = cv2.resize(img6,(256,256))
            # pcd_load6 = convert_imgTo_pcd(img6)
            # ros_cloud6 = sheet_array_to_pointcloud2(pcd_load6,-90,43)
            # self.image_pub6.publish(ros_cloud6)

            # img7 = mpimg.imread('parkpic67.jpg')
            # img7 = cv2.resize(img7,(256,256))
            # pcd_load7 = convert_imgTo_pcd(img7)
            # ros_cloud7 = sheet_array_to_pointcloud2(pcd_load7,0,86)
            # self.image_pub7.publish(ros_cloud7)

            # img8 = mpimg.imread('parkpic68.jpg')
            # img8 = cv2.resize(img8,(256,256))
            # pcd_load8 = convert_imgTo_pcd(img8)
            # ros_cloud8 = sheet_array_to_pointcloud2(pcd_load8,-57.3,86.4)
            # self.image_pub8.publish(ros_cloud8)

            # img9 = mpimg.imread('parkpic69.jpg')
            
            
            img9 = cv2.imread('parkpic7.jpg')
            hh, ww = img9.shape[:2]

            # threshold on white
            # Define lower and uppper limits
            lower = np.array([200, 200, 200])
            upper = np.array([255, 255, 255])
            
            thresh = cv2.inRange(img9, lower, upper)
            img9 = cv2.imwrite('parkpic72.jpg', thresh)
            img9 = cv2.imread('parkpic72.jpg')

            img9 = cv2.resize(img9,(746,746))
            # img9 = img9[0:868, 0:868]
            pcd_load9 = convert_imgTo_pcd(img9)
            ros_cloud9 = sheet_array_to_pointcloud2(pcd_load9,-89.2,86.4)
            self.image_pub9.publish(ros_cloud9)

            print("hello---------------1")
            
            rate.sleep()
        
        # self.callback()
        # rospy.spin()
    
    # def callback(self):
        
        
        
#---------------------------main---------------------------------
if __name__=="__main__":
    
    # rp = rpScanReceiver()
    ip = imageReceiver()