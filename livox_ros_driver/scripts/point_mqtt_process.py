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



def do_statistical_outlier_filtering(pcl_data,mean_k,tresh):
    '''
    :param pcl_data: point could data subscriber
    :param mean_k:  number of neighboring points to analyze for any given point
    :param tresh:   Any point with a mean distance larger than global will be considered outlier
    :return: Statistical outlier filtered point cloud data
    eg) cloud = do_statistical_outlier_filtering(cloud,10,0.001)
    : https://github.com/fouliex/RoboticPerception
    '''
    outlier_filter = pcl_data.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(mean_k)
    outlier_filter.set_std_dev_mul_thresh(tresh)
    return outlier_filter.filter()

def array_to_pointcloud2(cloud_arr, stamp=None, frame_id=None):
    '''Converts a numpy record array to a sensor_msgs.msg.PointCloud2.
    '''
    # ----------------------------------------------------------------------------------------
    points = []
    
    for i in range(cloud_arr.shape[0]):
        # for j in range(cloud_arr.size):
        #     for k in range(cloud_arr.size):

                x = cloud_arr[i][0]
                y = cloud_arr[i][1]
                z = cloud_arr[i][2]
                pt = [x,y,z,0]
                r = 0
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

#----------------------------------------------------------------------------------------------------------------------------------------

def getMarkerWindow(x,y,z,r,g,b):

	myMarker = Marker()
	myMarker.header.frame_id = "output"
	myMarker.header.seq = 1
	myMarker.header.stamp = rospy.get_rostime()
	myMarker.ns = "window"
	myMarker.id = 1
	myMarker.type = myMarker.POINTS
	# myMarker.action = myMarker.ADD
	# myMarker.pose.position.x = x
	# myMarker.pose.position.y = y
	myMarker.points.append(Point(x,y,z))
	q = quaternion_from_euler(0, 0, 0)
	myMarker.pose.orientation.x= q[0]
	myMarker.pose.orientation.y= q[1]
	myMarker.pose.orientation.z= q[2]
	myMarker.pose.orientation.w= 1
	# myMarker.mesh_resource = "package://project/models/window_buena.stl"
	myMarker.color.r, myMarker.color.g, myMarker.color.b , myMarker.color.a= r,g,b,1
	myMarker.scale.x = 1.5
	myMarker.scale.y = 4
	myMarker.scale.z = 0

	return myMarker

def base64UrlEncode(data):
    return urlsafe_b64encode(data).rstrip(b'=')
 
 
def base64UrlDecode(base64Url):
    padding = b'=' * (4 - (len(base64Url) % 4))
 
    return urlsafe_b64decode(base64Url + padding)

# liDAR data receiving class
class rpScanReceiver():
    
    def __init__(self):
        rospy.init_node('test', anonymous=True)
        
        self.sub = rospy.Subscriber('/livox/lidar', PointCloud2, self.callback)
        self.sub = rospy.Subscriber('/livox/imu', Imu, self.callbackImu)
        
        self.pub1 = rospy.Publisher('/marker2', Marker, queue_size=1)
        self.pub2 = rospy.Publisher('/marker3', Marker, queue_size=1)
        self.pub = rospy.Publisher("/out3", PointCloud2, queue_size=1)
        self.pub3 = rospy.Publisher("/temp", PointCloud2, queue_size=1)
        rospy.spin()
    
    
    #downSampling function (Create a VoxelGrid filter object for a input point cloud)
    def do_voxel_grid_downssampling(self,pcl_data,leaf_size):
        vox = pcl_data.make_voxel_grid_filter()
        vox.set_leaf_size(leaf_size, leaf_size, leaf_size) # The bigger the leaf size the less information retained
        return  vox.filter()
    
    def do_passthrough(self, pcl_data,filter_axis,axis_min,axis_max):
        passthrough = pcl_data.make_passthrough_filter()
        passthrough.set_filter_field_name(filter_axis)
        passthrough.set_filter_limits(axis_min, axis_max)
        return passthrough.filter()

    def do_ransac_plane_normal_segmentation(self, point_cloud, input_max_distance):
        segmenter = point_cloud.make_segmenter_normals(ksearch=50)
        segmenter.set_optimize_coefficients(True)
        segmenter.set_model_type(pcl.SACMODEL_NORMAL_PLANE)  #pcl_sac_model_plane
        segmenter.set_normal_distance_weight(0.1)
        segmenter.set_method_type(pcl.SAC_RANSAC) #pcl_sac_ransac
        segmenter.set_max_iterations(1000)
        segmenter.set_distance_threshold(input_max_distance) #0.03)  #max_distance
        indices, coefficients = segmenter.segment()

        inliers = point_cloud.extract(indices, negative=False)
        outliers = point_cloud.extract(indices, negative=True)

        return indices, inliers, outliers
 
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
        
        
        #mqtt = mq.Client("mypub")
        #mqtt.connect("localhost", 1883)
        #imu_json = json.dumps(imu_msg)
        #print(json.loads(imu_json))
        #mqtt.publish("mqtt/imu",imu_json)
        
        #print("The imu message is published.")
        #mqtt.loop(2)
        
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
        
        #get acceleration data from livox horizon
        x = -1 * imu.linear_acceleration.x 
        y = -1 * imu.linear_acceleration.y
        z = -1 * imu.linear_acceleration.z
        
        # acceleration's direction is opposite direction of gravity
        # scalar of sum vector
        g = math.sqrt(abs(x)*abs(x) + abs(y)*abs(y) + abs(z)*abs(z))

        #angle vector g to axis
        #theta_x = math.acos(x/g)
        #theta_y = math.acos(y/g)
        #theta_z = math.acos(z/g)

        
        #theta rotate by y axis 
        theta_y_trans = (theta_y_trans * 99 + math.acos(-z/math.sqrt(x*x + z*z))) / 100
        #theta rotate by x axis
        theta_x_trans = (theta_x_trans * 99 + -1 * math.acos(math.sqrt(x*x+z*z)/g)) / 100

        #print("y_axis rotation : ", theta_y_trans * 57.29577951308232, " // x_axis rotation : ", theta_x_trans * 57.29577951308232)
        


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
        edata = str(point_dic1)
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
        
        
        ##  --------------------------------------- downsampling  -------------------------##
        print("before downsampling:", cloud)
        LEAF_SIZE = 0.2
        cloud = self.do_voxel_grid_downssampling(cloud, LEAF_SIZE)
        print("after downsampling:", cloud)
        print("")

        ##  -----------------Statistical Outlier Removal ---------------##
        #filter_axis = 'x'
        #axis_min = 0.0
        #axis_max = 100.0
        #cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)
        #
        #filter_axis = 'y'
        #axis_min = -10.0
        #axis_max = 10.0
        #cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)
        #
        #filter_axis = 'z'
        #axis_min = -2.0
        #axis_max = 2.0
        #cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)
        
        ##  ----------------- Statistical Outlier Removal ---------------##
        
        cloud = self.do_passthrough(cloud, 'x', 1.0, 60.0)
        cloud = self.do_passthrough(cloud, 'y', -7.0, 5.5)
        _, floor, cloud = self.do_ransac_plane_normal_segmentation(cloud, 0.05)
        
        ##  ----------------- Statistical Outlier Removal ---------------##
        
        
        #cloud = floor
        cloud_arr = np.asarray(cloud)
        
        
        ##------------------------------------ rotate data -----------------------------------------------------##
        #theta_y_trans : yaw
        #theta_x_trans : pitch
        global theta_y_trans
        global theta_x_trans

        ## yaw(y-axis) rotate
        for point in range(cloud.size):
            x = cloud_arr[point][0]
            z = cloud_arr[point][2]

            cloud_arr[point][0] = math.cos(theta_y_trans + theta_y_offset) * x + math.sin(theta_y_trans + theta_y_offset) * z
            cloud_arr[point][2] = -1 * math.sin(theta_y_trans + theta_y_offset) * x + math.cos(theta_y_trans + theta_y_offset) * z
        
        ## pitch(x-axis) rotate
        for point in range(cloud.size):
            y = cloud_arr[point][1]
            z = cloud_arr[point][2]
            
            cloud_arr[point][1] = (math.cos(theta_x_trans + theta_x_offset) * y) - (math.sin(theta_x_trans + theta_x_offset) * z)
            cloud_arr[point][2] = (math.sin(theta_x_trans + theta_x_offset) * y) + (math.cos(theta_x_trans + theta_x_offset) * z)
        ##------------------------------------ rotate data -----------------------------------------------------##
        
        
        ## ------------------------------------ delete ceiling --------------------------------------------- ##
        tmp_msg = array_to_pointcloud2(cloud_arr, rospy.Time.now(), frame_id='output')
        cloud = ros_to_pcl(tmp_msg)
        #cloud = self.do_passthrough(cloud, 'z', -1, 3)
        after = np.asarray(cloud)
        tmp_msg = array_to_pointcloud2(after, rospy.Time.now(), frame_id='output')
        self.pub3.publish(tmp_msg)
        ## ------------------------------------ delete ceiling --------------------------------------------- ##
        
        ## --------------- process --------------------##
        count_1 = 0
        count_2 = 0
        flag_1 = True
        flag_2 = True
        
        ##cloud.from_array(cloud_arr)
        #for point in range(cloud.size):
        #    x = cloud_arr[point][0]
        #    y = cloud_arr[point][1]
        #    z = cloud_arr[point][2]
        #    
        #    if z > 0:
        #         cloud_arr[point][0] = 0
        #         cloud_arr[point][1] = 0
        #         cloud_arr[point][2] = 0
        #    
        #    if x > 6.5 and x < 7.7 and y > 1.3 and y < 5 and z > -1.2 and z < 0.4:
        #        count_1 += 1
        #    if x > 3.8 and x < 5.8 and y > 1.3 and y < 5 and z > -1.2 and z < 0.4:
        #        count_2 += 1
        ##print(count_1)
        ##print(count_2)
        #
        #    
        #if count_1 > 10:
        #    #print("1 is not empty")
        #    flag_1 = False
        #else:
        #    #print("1 is empty!")
        #    flag_1 = True
        #if count_2 > 10:
        #    #print("2 is not empty")
        #    flag_2 = False
        #    self.pub1.publish(getMarkerWindow(5, 3.5, -1.4, 0,0,1))
        #    # self.pub2.publish(getMarkerWindow(8, 4, -1.4, 0,0,1))
        #else:
        #    #print("2 is empty!")
        #    flag_2 = True
        #    self.pub1.publish(getMarkerWindow(5, 3.5, -1.4, 1,0,0))
        #    # self.pub2.publish(getMarkerWindow(8, 4, -1.4, 1,0,0))
        
        ## --------------- process --------------------##
        print("1111111111")
        ## ------------------- inout publish --------------------- ##
        mqtt = mq.Client("mypub")

        mqtt.connect("localhost", 1883)
        
        inout_msg = {}
        plist = ["park_1","park_2"]
        flist = [flag_1, flag_2]
        for index in range(len(plist)):
            inout_msg[plist[index]] = flist[index]
        #inout_json = json.dumps(inout_msg)
        #mqtt.publish("mqtt/inout", inout_json)
        #mqtt.publish("mqtt/inout/park_1",flag_1)
        #mqtt.publish("mqtt/inout/park_2",flag_2)
        #print("The inout message is published.")
        #mqtt.loop(2)
       
        ## ------------------- inout publish --------------------- ##
        
        ## ---------------------------------- mqtt inout send to server ------------------------------------##
        
        service_mqtt_client = mqtt_data_client(logins, topic_class)

        client_id = logins.clientID

        ## publish topic : wlogs/device/service/wlogs:kor:wlogsORG:embedded-sg20:137fcf3c-70b7-4b81-835e-c64518dab3fc:1658896328.073243/lidar/sktv1/inout
        service_topic = "wlogs/device/service/" +client_id+ "/" + topic_class + "/" + location + "/inout" # /wlogs/device/service/{device_id}/{version}/api/{parkingLotId}/log
        #print("@@@", service_topic)
        #service_mqtt_client.publish(
        #        topic=service_topic,
        #        msg=inout_json
        #    )
        #publish_pks_data(service_mqtt_client, service_topic, inout_msg, parkingFloor)
        inout_list = []
        #cp = check_token_is_expired(service_mqtt_client)
        #service_mqtt_client.password = cp
        #service_mqtt_client.stop()
        #service_mqtt_client.setUser(pwd=cp)
        #service_mqtt_client.start()
        #start = time.time()
        inout_list.append(inout_msg)
        service_mqtt_client.publish(service_topic, {"pkslog" : inout_list})
        time.sleep(1)
        #service_mqtt_client.subscribe(topic = service_topic)
        #print()
        ## ---------------------------------- mqtt pcd send to server ------------------------------------##
        
        
        ##  --------------------------------------- floor trash  -------------------------##
        
        #pcl.save(cloud,"pcdex.pcd")
        #test1 = o3d.io.read_point_cloud("pcdex.pcd")
        #o3d.visualization.draw_geometries([test1])
        
        #cloud = self.do_passthrough(cloud, 'x', 1.0, 20.0)
        #cloud = self.do_passthrough(cloud, 'y', -7.0, 5.5)
        #_, floor, cloud = self.do_ransac_plane_normal_segmentation(cloud, 0.05)
        
        #pcl.save(cloud,"pcdex.pcd")
        #test1 = o3d.io.read_point_cloud("pcdex.pcd")
        #o3d.visualization.draw_geometries([test1])
    
        #pcl.save(floor,"livox_horizon_floor2.pcd")
        #test1 = o3d.io.read_point_cloud("livox_horizon_floor2.pcd")
        #o3d.visualization.draw_geometries([test1])
        
        # pcl.save(cloud,"livox_horizon.pcd")
        # pcl.save(cloud,"lanedetection.pcd")
        ##  --------------------------------------- floor trash  -------------------------##
        
        
        
        #numpy array를 ROS 메시지로 변경
        tmp_msg = PointCloud2()
        tmp_msg = array_to_pointcloud2(cloud_arr, rospy.Time.now(), frame_id='output')
        self.pub.publish(tmp_msg)

#------------------------------main---------------------------------
if __name__=="__main__":
    global logins
    print("login to DB server")
    logins = login_mqtt()
    
    rp = rpScanReceiver()