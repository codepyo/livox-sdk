#!/usr/bin/env python

from re import X
from tkinter import Y
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
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Imu
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point
from random import randint
import math
import ros_numpy
import sensor_msgs

#-*- coding:utf-8 -*-
import paho.mqtt.client as mq
import json

import base64
from base64 import urlsafe_b64encode, urlsafe_b64decode
import zlib
import random
import string

## --- MQTT to server --- ##
from mqtt.WatchMileMQTT import WatchMileMQTT
from mqtt.Login_MQTT import login_mqtt
from mqtt.Service_MQTT import mqtt_data_client
import time

theta_y_trans = 1.57
theta_x_trans = 0
theta_y_offset = -0.025
theta_x_offset = 0

# Converts a ROS PointCloud2 message to a pcl PointXYZRGB
def ros_to_pcl(ros_cloud):
    points_list = []
    color_list = []

    for data in pc2.read_points(ros_cloud, skip_nans=True):
        points_list.append([data[0], data[1], data[2]])
        
        #color_list = float_to_rgb(data[3])

    pcl_data = pcl.PointCloud()
    pcl_data.from_list(points_list)

    return pcl_data


# liDAR data receiving class
class LidarMQTT():
    
    def __init__(self):
        global logins
        print("login to DB server")
        logins = login_mqtt()
        
        rospy.init_node('test', anonymous=True)
        
        self.sub = rospy.Subscriber('/livox/lidar', PointCloud2, self.callback)
        self.sub = rospy.Subscriber('/inout', String, self.callbackInout)
        self.sub = rospy.Subscriber('/livox/imu', Imu, self.callbackImu)
        
        #self.pub = rospy.Publisher("/out3", PointCloud2, queue_size=1)  
        rospy.spin()
    
 
    def callbackInout(self, input_ros_msg):
        inout = eval(input_ros_msg.data)
        topic_class='lidar'
        location = 'skv1'
        parkingFloor = 'b3'
        ## ---------------------------------- mqtt inout send to server ------------------------------------##
        
        service_mqtt_client = mqtt_data_client(logins, topic_class)
        client_id = logins.clientID
        ## publish topic : wlogs/device/service/wlogs:kor:wlogsORG:embedded-sg20:137fcf3c-70b7-4b81-835e-c64518dab3fc:1658896328.073243/lidar/sktv1/inout
        service_topic = "wlogs/device/service/" +client_id+ "/" + topic_class + "/" + location + "/inout" # /wlogs/device/service/{device_id}/{version}/api/{parkingLotId}/log
        inout_list = []
        inout_list.append(inout)
        service_mqtt_client.publish(service_topic, {"pkslog" : inout_list})
        time.sleep(1)
        ## ---------------------------------- mqtt pcd send to server ------------------------------------##
 
    ## calculate theta to rotate data ##
    def callbackImu(self, input_ros_msg):
        global theta_y_trans
        global theta_x_trans
        imu = input_ros_msg
        
        ## --------------------- mqtt acc data send --------------------##
        #imu_msg = {"x":imu.linear_acceleration.x, "y":imu.linear_acceleration.y, "z":imu.linear_acceleration.z}
        imu_msg = {}
        imu_msg["x"] = imu.linear_acceleration.x
        imu_msg["y"] = imu.linear_acceleration.y
        imu_msg["z"] = imu.linear_acceleration.z
        
        imu_list = []
        topic_class='lidar'
        location = 'skv1'
        parkingFloor = 'b3'
        service_mqtt_client = mqtt_data_client(logins, topic_class)

        client_id = logins.clientID

        ## publish topic : wlogs/device/service/wlogs:kor:wlogsORG:embedded-sg20:137fcf3c-70b7-4b81-835e-c64518dab3fc:1658890799.722915/lidar/sktv1/imu
        service_topic = "wlogs/device/service/" +client_id+ "/" + topic_class + "/" + location + "/imu" # /wlogs/device/service/{device_id}/{version}/api/{parkingLotId}/log
        #print("@@@", service_topic)
        #service_mqtt_client.subscribe(topic = service_topic + "/result")
        imu_list.append(imu_msg)
        service_mqtt_client.publish(service_topic, {"pkslog" : imu_list})
        time.sleep(1)
        
        ## --------------------- mqtt acc data send --------------------##


    def callback(self, input_ros_msg):
        cloud = ros_to_pcl(input_ros_msg)
        
        ## ---------------------------------- mqtt pcd send data ------------------------------------##
        #mqtt = mq.Client("mypub")
        #mqtt.connect("localhost", 1883) 
        #pcd_json = json.dumps(str(input_ros_msg))
        pcd_msg = {}
        pcd_header = {}
        pcd_header_stamp = {}
        pcd_header["seq"] = input_ros_msg.header.seq
        pcd_header_stamp["secs"] = input_ros_msg.header.stamp.secs
        pcd_header_stamp["nsecs"] = input_ros_msg.header.stamp.nsecs
        pcd_header["stamp"] = pcd_header_stamp
        pcd_header["frame_id"] = input_ros_msg.header.frame_id

        pcd_fields = {}
        pcd_fields["x"] = {"offset": 0, "datatype": 7, "count": 1}
        pcd_fields["y"] = {"offset": 4, "datatype": 7, "count": 1}
        pcd_fields["z"] = {"offset": 8, "datatype": 7, "count": 1}
        pcd_fields["intensity"] = {"offset": 12, "datatype": 7, "count": 1}
        pcd_fields["tag"] = {"offset": 16, "datatype": 2, "count": 1}
        pcd_fields["line"] = {"offset": 17, "datatype": 2, "count": 1}
        
        pcd_msg["header"] = pcd_header
        pcd_msg["height"] = input_ros_msg.height
        pcd_msg["width"] = input_ros_msg.width
        pcd_msg["fields"] = pcd_fields
        pcd_msg["bigendian"] = input_ros_msg.is_bigendian
        pcd_msg["point_step"] = input_ros_msg.point_step
        pcd_msg["row_step"] = input_ros_msg.row_step
        #pcd_msg["data"] = str(base64.b64encode(input_ros_msg.data))
        pcd_msg["is_dense"] = input_ros_msg.is_dense
        
        #input_ros_msg.data = base64.b64decode(base64.b64encode(input_ros_msg.data))
        #pcd_msg["data"] = str(input_ros_msg.data)
        #point_list = []
        point_dic1 = {}
        point_dic2 = {}
        point_dic3 = {}
        point_dic4 = {}
        point_dic5 = {}
        
        i = 1
        for data in pc2.read_points(input_ros_msg, skip_nans=True):
            if i < 5000:
                point_dic1["point" + str(i)] = {"x" : data[0], "y" : data[1], "z" : data[2], "I" : data[3], "tag" : data[4], "line" : data[5]}
            elif i < 10000:
                point_dic2["point" + str(i)] = {"x" : data[0], "y" : data[1], "z" : data[2], "I" : data[3], "tag" : data[4], "line" : data[5]}
            elif i < 15000:
                point_dic3["point" + str(i)] = {"x" : data[0], "y" : data[1], "z" : data[2], "I" : data[3], "tag" : data[4], "line" : data[5]}
            elif i < 20000:
                point_dic4["point" + str(i)] = {"x" : data[0], "y" : data[1], "z" : data[2], "I" : data[3], "tag" : data[4], "line" : data[5]}
            else:
                point_dic5["point" + str(i)] = {"x" : data[0], "y" : data[1], "z" : data[2], "I" : data[3], "tag" : data[4], "line" : data[5]}
            i += 1
        
        ## ---------------------------------- mqtt pcd send to server ------------------------------------##
        
        topic_class='lidar'
        location = 'skv1'
        parkingFloor = 'b3'
        
        ## publish topic : wlogs/device/service/wlogs:kor:wlogsORG:embedded-sg20:137fcf3c-70b7-4b81-835e-c64518dab3fc:1658890685.927806/lidar/sktv1/pointcloud
        service_mqtt_client = mqtt_data_client(logins, topic_class)
        client_id = logins.clientID
        pcd_list = []
        edata = str(point_dic1).encode('utf-8').hex()
        #zdata = zlib.compress(str(point_dic1).encode('utf-8'), -1)
        pcd_msg["data"] = str(edata)
        #pcd_msg["data"] = str(point_dic1)
        pcd_list.append(pcd_msg)
        service_topic = "wlogs/device/service/" +client_id+ "/" + topic_class + "/" + location + "/pointcloud/1" # /wlogs/device/service/{device_id}/{version}/api/{parkingLotId}/log
        service_mqtt_client.publish(service_topic, {"pksnum": 1, "pkstime" : time.strftime('%x %X'),"pkslog" : pcd_list})
        
        service_mqtt_client = mqtt_data_client(logins, topic_class)
        client_id = logins.clientID
        pcd_list = []
        edata = str(point_dic2)
        #zdata = zlib.compress(str(point_dic2).encode('utf-8'), -1)
        pcd_msg["data"] = str(edata)
        #pcd_msg["data"] = str(point_dic2)
        pcd_list.append(pcd_msg)
        service_topic = "wlogs/device/service/" +client_id+ "/" + topic_class + "/" + location + "/pointcloud/2"
        service_mqtt_client.publish(service_topic, {"pksnum": 2, "pkstime" : time.strftime('%x %X'),"pkslog" : pcd_list})
        
        service_mqtt_client = mqtt_data_client(logins, topic_class)
        client_id = logins.clientID
        pcd_list = []
        edata = str(point_dic3)
        #zdata = zlib.compress(str(point_dic3).encode('utf-8'), -1)
        pcd_msg["data"] = str(edata)
        #pcd_msg["data"] = str(point_dic3)
        pcd_list.append(pcd_msg)
        service_topic = "wlogs/device/service/" +client_id+ "/" + topic_class + "/" + location + "/pointcloud/3"
        service_mqtt_client.publish(service_topic, {"pksnum": 3, "pkstime" : time.strftime('%x %X'), "pkslog" : pcd_list})
        
        service_mqtt_client = mqtt_data_client(logins, topic_class)
        client_id = logins.clientID
        pcd_list = []
        edata = str(point_dic4)
        #zdata = zlib.compress(str(point_dic4).encode('utf-8'), -1)
        pcd_msg["data"] = str(edata)
        #pcd_msg["data"] = str(point_dic4)
        pcd_list.append(pcd_msg)
        service_topic = "wlogs/device/service/" +client_id+ "/" + topic_class + "/" + location + "/pointcloud/4"
        service_mqtt_client.publish(service_topic, {"pksnum": 4, "pkstime" : time.strftime('%x %X'),"pkslog" : pcd_list})
        
        service_mqtt_client = mqtt_data_client(logins, topic_class)
        client_id = logins.clientID
        pcd_list = []
        edata = str(point_dic5)
        #zdata = zlib.compress(str(point_dic5).encode('utf-8'), -1)
        pcd_msg["data"] = str(edata)
        #pcd_msg["data"] = str(point_dic4)
        pcd_list.append(pcd_msg)
        service_topic = "wlogs/device/service/" +client_id+ "/" + topic_class + "/" + location + "/pointcloud/5"
        service_mqtt_client.publish(service_topic, {"pksnum": 5, "pkstime" : time.strftime('%x %X'),"pkslog" : pcd_list})
        
        #f = open("newfile.txt", 'w')
        #data = str(pcd_msg)
        #
        #f.write(data)
        #f.close()
        
        ## ---------------------------------- mqtt pcd send to server ------------------------------------##


#------------------------------main---------------------------------
if __name__=="__main__":
    lidar_mqtt = LidarMQTT()