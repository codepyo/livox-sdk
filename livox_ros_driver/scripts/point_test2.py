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

theta_y_trans = 1.57
theta_x_trans = 0

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
	q = quaternion_from_euler(1.57, 0, 0)
	myMarker.pose.orientation.x=0
	myMarker.pose.orientation.y=0
	myMarker.pose.orientation.z=0
	myMarker.pose.orientation.w=1
	# myMarker.mesh_resource = "package://project/models/window_buena.stl"
	myMarker.color.r, myMarker.color.g, myMarker.color.b , myMarker.color.a= r,g,b,1
	myMarker.scale.x = 1
	myMarker.scale.y = 1
	myMarker.scale.z = 0

	return myMarker


# liDAR data receiving class
class rpScanReceiver():
    
    def __init__(self):
        rospy.init_node('test', anonymous=True)
        
        self.sub = rospy.Subscriber('/livox/lidar', PointCloud2, self.callback)
        self.sub = rospy.Subscriber('/livox/imu', Imu, self.callbackImu)
        
        self.pub1 = rospy.Publisher('/marker', Marker, queue_size=1)
        self.pub = rospy.Publisher("/out", PointCloud2, queue_size=1)
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
        theta_y_trans = math.acos(-z/math.sqrt(x*x + z*z))
        theta_x_trans = -1 * math.acos(math.sqrt(x*x+z*z)/g)

        #print("y_axis rotation : ", theta_y_trans * 57.29577951308232, " // x_axis rotation : ", theta_x_trans * 57.29577951308232)
        


    def callback(self, input_ros_msg):
        cloud = ros_to_pcl(input_ros_msg)
        
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
        
        cloud = self.do_passthrough(cloud, 'x', 1.0, 20.0)
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
        
            cloud_arr[point][0] = math.cos(theta_y_trans) * x + math.sin(theta_y_trans) * z
            cloud_arr[point][2] = -1 * math.sin(theta_y_trans) * x + math.cos(theta_y_trans) * z
        
        ## pitch(x-axis) rotate
        for point in range(cloud.size):
            y = cloud_arr[point][1]
            z = cloud_arr[point][2]
            
            cloud_arr[point][1] = (math.cos(theta_x_trans) * y) - (math.sin(theta_x_trans) * z)
            cloud_arr[point][2] = (math.sin(theta_x_trans) * y) + (math.cos(theta_x_trans) * z)
        ##------------------------------------ rotate data -----------------------------------------------------##
        
        
        ## --------------- perception --------------------##
        count_1 = 0
        count_2 = 0
        #cloud.from_array(cloud_arr)
        for point in range(cloud.size):
            x = cloud_arr[point][0]
            y = cloud_arr[point][1]
            z = cloud_arr[point][2]
            
            if z > 0.6:
                 cloud_arr[point][0] = 0
                 cloud_arr[point][1] = 0
                 cloud_arr[point][2] = 0
            
            if x > 6.5 and x < 7.7 and y > 1.3 and y < 5 and z > -1.2 and z < 0.4:
                count_1 += 1
            if x > 3.8 and x < 5.8 and y > 1.3 and y < 5 and z > -1.2 and z < 0.4:
                count_2 += 1
                            
        if count_1 > 10:
            print("1 is not empty")
        else:
            print("1 is empty!")
        if count_2 > 10:
            self.pub1.publish(getMarkerWindow(5.1, 5, -1.4, 1,1,1))
            print("2 is not empty")
        else:
            self.pub1.publish(getMarkerWindow(5.1, 5, -1.4, 1,1,0))
            print("2 is empty!")
        
        ## --------------- perception --------------------##
        
        
       
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
    
    rp = rpScanReceiver()