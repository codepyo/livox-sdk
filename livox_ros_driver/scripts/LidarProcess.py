#!/usr/bin/env python
from re import X
from tkinter import Y
import pcl
#import pcl_helper

import numpy as np
import open3d as o3d
import struct
import rospy
import cv2
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point
from random import randint
import math

theta_y_trans = 1.57
theta_x_trans = 0
theta_y_offset = -0.025
theta_x_offset = -0.04
theta_z_offset = -0.02

time1 = 0
time2 = 0

# The data structure of each point in ros PointCloud2: 16 bits = x + y + z + rgb
FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
FIELDS_XYZRGB = FIELDS_XYZ + \
    [PointField(name='rgb', offset=16, datatype=PointField.UINT32, count=1)]

# Bit operations
BIT_MOVE_16 = 2**16
BIT_MOVE_8 = 2**8

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

# def convertCloudFromOpen3dToRos(open3d_cloud, frame_id="output"):
def convertCloudFromOpen3dToRos(open3d_cloud, frame_id="output"):
    # Set "header"
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id

    # Set "fields" and "cloud_data"
    points=np.asarray(open3d_cloud.points)
    if not open3d_cloud.colors: # XYZ only
        fields=FIELDS_XYZ
        cloud_data=points
    else: # XYZ + RGB
        
        print("--------------------------------------")
        fields=FIELDS_XYZRGB
    # -- Change rgb color from "three float" to "one 24-byte int"
    # 0x00FFFFFF is white, 0x00000000 is black.
    
        colors = np.floor(np.asarray(open3d_cloud.colors)*255) # nx3 matrix
        print("--------------------------------------")
        colors = colors[:,0] * BIT_MOVE_16 +colors[:,1] * BIT_MOVE_8 + colors[:,2]
        print("--------------------------------------")  
        cloud_data=np.c_[points, colors]
        print("--------------------------------------")
    
    # create ros_cloud
    return pc2.create_cloud(header, fields, cloud_data)


def distort_pcd(my_img_position, **kwargs):
	l = my_img_position.shape[0] 
	t = np.linspace(0,1,l,endpoint = False)
	#sig = 0.05 * np.sin(8*np.pi*t)	
	sig = np.sin(0.8*np.pi*t)	
	#t = np.linspace(-0.5,0.5,l,endpoint = False)
	#sig = 0.1 * np.sinc(8*np.pi*t)
	my_img_position[:,2] = sig
	return my_img_position


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
 




def do_statistical_outlier_filtering(pcl_data,mean_k,tresh):
    
    outlier_filter = pcl_data.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(mean_k)
    outlier_filter.set_std_dev_mul_thresh(tresh)
    return outlier_filter.filter()

def array_to_pointcloud2(cloud_arr, stamp=None, frame_id=None):
    '''Converts a numpy record array to a sensor_msgs.msg.PointCloud2.
    '''
    
    points = []
    
    for i in range(cloud_arr.shape[0]):
        # for j in range(cloud_arr.size):
        #     for k in range(cloud_arr.size):

        x = cloud_arr[i][0]
        y = cloud_arr[i][1]
        z = cloud_arr[i][2]
        
        pt = [x,y,z,0]
        r = 216
        g = 52
        b = 235
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


# def sheet_array_to_pointcloud2(point, colors, stamp=None, frame_id=None):
def sheet_array_to_pointcloud2(cloud_arr, nx, ny, stamp=None, frame_id=None):
    
    point = np.asarray(cloud_arr.points)
    colors = np.asarray(cloud_arr.colors)
    points = []
    
    for i in range(60000):

        x = point[i][0] * 45.0 + nx #59.7
        y = point[i][1] * 45.0 + ny #14
        z = point[i][2] * 45.0 
        
        pt = [x,y,z,0]
        
        r = int(colors[i][0]*255.0)
        g = int(colors[i][1]*255.0)
        b = int(colors[i][2]*255.0)
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
	myMarker.pose.orientation.w= 0
	# myMarker.mesh_resource = "package://project/models/window_buena.stl"
	myMarker.color.r, myMarker.color.g, myMarker.color.b , myMarker.color.a= r,g,b,1
	myMarker.scale.x = 1.5
	myMarker.scale.y = 4
	myMarker.scale.z = 0

	return myMarker


# liDAR data receiving class
class rpScanReceiver():
    
    def __init__(self, opt):
        rospy.init_node('test', anonymous=True)
        
        self.sub = rospy.Subscriber('/livox/lidar', PointCloud2, self.callback)
        self.sub = rospy.Subscriber('/livox/imu', Imu, self.callbackImu)
        
        self.pub = rospy.Publisher("/out", PointCloud2, queue_size=1)
        self.pub2 = rospy.Publisher("/inout", String, queue_size=1)
        self.pub3 = rospy.Publisher("/temp", PointCloud2, queue_size=1)
        self.pub1 = rospy.Publisher('/marker1', Marker, queue_size=1)
        self.pub4 = rospy.Publisher('/marker2', Marker, queue_size=1)
        
        
        
        self.voxel = opt.downsampling
        
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
        theta_y_trans = (theta_y_trans * 99 + math.acos(-z/math.sqrt(x*x + z*z))) / 100
        #theta rotate by x axis
        theta_x_trans = (theta_x_trans * 99 + -1 * math.acos(math.sqrt(x*x+z*z)/g)) / 100

        #print("y_axis rotation : ", theta_y_trans * 57.29577951308232, " // x_axis rotation : ", theta_x_trans * 57.29577951308232)
        
    def callback(self, input_ros_msg):
        cloud = ros_to_pcl(input_ros_msg)
        global time1
        global time2
        ##  --------------------------------------- downsampling  -----------------------------------------------------##
        print("before downsampling:", cloud)
        LEAF_SIZE = 0.2
        #cloud = self.do_voxel_grid_downssampling(cloud, LEAF_SIZE)
        cloud = self.do_voxel_grid_downssampling(cloud, self.voxel)
        print("after downsampling:", cloud)
        print("")
        
        ##  --------------------------------------- downsampling  -----------------------------------------------------##

        
        ##  -------------------------------------- Statistical Outlier Removal -------------------------------------------##
        
        cloud = self.do_passthrough(cloud, 'x', 0.0, 50.0)
        cloud = self.do_passthrough(cloud, 'y', -15.0, 15.0)
        _, floor, cloud = self.do_ransac_plane_normal_segmentation(cloud, 0.1)
        
        ##  -------------------------------------- Statistical Outlier Removal -------------------------------------------##
        
        
        #cloud = floor
        cloud_arr = np.asarray(cloud)
        
        
        ##------------------------------------ rotate data -----------------------------------------------------##
        #theta_y_trans : yaw
        #theta_x_trans : pitch
        global theta_y_trans
        global theta_x_trans
        global theta_z_offset
        
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
            
        for point in range(cloud.size):
            x = cloud_arr[point][0]
            y = cloud_arr[point][1]
            
            cloud_arr[point][0] = (math.cos(theta_z_offset) * x - math.sin(theta_z_offset) * y)
            cloud_arr[point][1] = (math.sin(theta_z_offset) * x + math.cos(theta_z_offset) * y)
            
            
        ##------------------------------------ rotate data -----------------------------------------------------##
        
        
        ## ------------------------------------ delete ceiling --------------------------------------------- ##
        # tmp_msg = array_to_pointcloud2(cloud_arr, rospy.Time.now(), frame_id='output')
        # cloud = ros_to_pcl(tmp_msg)
        # cloud = self.do_passthrough(cloud, 'z', -3.0, -0.8)
        # print(cloud)
        # after = np.asarray(cloud)
        # tmp_msg = array_to_pointcloud2(after, rospy.Time.now(), frame_id='output')
        # self.pub3.publish(tmp_msg)
        ## ------------------------------------ delete ceiling --------------------------------------------- ##
        
        
        ## -------------------------------------- process ----------------------------------------------------------##
        count_1 = 0
        count_2 = 0
        flag_1 = True
        flag_2 = True
        #cloud.from_array(cloud_arr)
        minz = 100
        for point in range(cloud.size):
            x = cloud_arr[point][0]
            y = cloud_arr[point][1]
            z = cloud_arr[point][2]
            
            # if(z<minz): minz = z
            
            ## -------------------------------------- ceil removal -----------------------------------##
            if z > -1:
                 cloud_arr[point][0] = 0
                 cloud_arr[point][1] = 0
                 cloud_arr[point][2] = 0
            
            # if z > 0.6:
            #     cloud_arr[point][0] = 0
            #     cloud_arr[point][1] = 0
            #     cloud_arr[point][2] = 0
            
            ## -------------------------------------- ceil removal -----------------------------------##
            
            if x > 16.5 and x < 18.5 and y > -15 and y < -11 and z > -3.5:
                count_1 += 1
            if x > 31.5 and x < 33.5 and y > 0 and y < 5 and z > -3.5:
                count_2 += 1
                
        # print("min z!!!:", minz)
        print(count_1)
        print(count_2)    
        
        
        if count_1 > 5:
            self.pub1.publish(getMarkerWindow(17.3, -13.5, -3.5, 0,0,1))
            print("1 is not empty")
            flag_1 = False
            time1 = 0
        elif time1 > 2:
            self.pub1.publish(getMarkerWindow(17.3, -13.5, -3.5, 1,0,0))
            print("1 is empty!")
            flag_1 = True
            time1 = 0
        else:
            time1 += 1
            
            
        if count_2 > 5:
            print("2 is not empty")
            flag_2 = False
            self.pub4.publish(getMarkerWindow(32.5, 2.25, -3.5, 0,0,1))
            time2 = 0
        elif time2 > 2:
            print("2 is empty!")
            flag_2 = True
            self.pub4.publish(getMarkerWindow(32.5, 2.25, -3.5, 1,0,0))
            time2 = 0
        else:
            time2 += 1
        
        ## --------------------------------------------- process ----------------------------------------------##
        
        ## ------------------- inout publish to mqtt module --------------------- ##
        inout_msg = {}
        plist = ["park_1","park_2"]
        flist = [flag_1, flag_2]
        for index in range(len(plist)):
            inout_msg[plist[index]] = flist[index]

        self.pub2.publish(str(inout_msg))
        ## ------------------- inout publish to mqtt module --------------------- ##
      
        ##  --------------------------------------- floor removal  -------------------------##
        
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
        ##  --------------------------------------- floor removal  -------------------------##
        
        
        #numpy array를 ROS 메시지로 변경
        tmp_msg = PointCloud2()
        tmp_msg = array_to_pointcloud2(cloud_arr, rospy.Time.now(), frame_id='output')
        self.pub.publish(tmp_msg)
        

#---------------------------main---------------------------------
if __name__=="__main__":
    
    rp = rpScanReceiver()
    # ip = imageReceiver()