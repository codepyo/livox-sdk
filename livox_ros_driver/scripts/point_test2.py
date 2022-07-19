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
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.msg import Imu
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from random import randint
import math
import ros_numpy
import sensor_msgs

theta_y_trans = 1.57
theta_x_trans = 0
# clash with any actual field names
DUMMY_FIELD_PREFIX = '__'

# sizes (in bytes) of PointField types
pftype_sizes = {PointField.INT8: 1, PointField.UINT8: 1, PointField.INT16: 2, PointField.UINT16: 2,
                PointField.INT32: 4, PointField.UINT32: 4, PointField.FLOAT32: 4, PointField.FLOAT64: 8}

# mappings between PointField types and numpy types
type_mappings = [(PointField.INT8, np.dtype('int8')), (PointField.UINT8, np.dtype('uint8')), (PointField.INT16, np.dtype('int16')),
                 (PointField.UINT16, np.dtype('uint16')), (PointField.INT32, np.dtype('int32')), (PointField.UINT32, np.dtype('uint32')),
                 (PointField.FLOAT32, np.dtype('float32')), (PointField.FLOAT64, np.dtype('float64'))]
pftype_to_nptype = dict(type_mappings)
nptype_to_pftype = dict((nptype, pftype) for pftype, nptype in type_mappings)

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
        points_list.append([data[0], data[1], data[2]])
        
        #color_list = float_to_rgb(data[3])

    pcl_data = pcl.PointCloud()
    pcl_data.from_list(points_list)

    return pcl_data

# Converts a pcl PointXYZRGB to a ROS PointCloud2 message

def pcl_to_ros(pcl_array):
    ros_msg = PointCloud2()

    ros_msg.header.stamp = rospy.Time.now()
    ros_msg.header.frame_id = "world"

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
    
    ros_msg.data = "".join(buffer)
    
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
        for i in range(len(get_color_list.color_list), cluster_count):
            get_color_list.color_list.append(random_color_gen())
    return get_color_list.color_list

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

def dtype_to_fields(dtype):
    '''Convert a numpy record datatype into a list of PointFields.
    '''
    fields = []
    print(dtype.fields)
    for field_name in dtype.names:
        np_field_type, field_offset = dtype.fields[field_name]
        pf = PointField()
        pf.name = field_name
        if np_field_type.subdtype:
            item_dtype, shape = np_field_type.subdtype
            pf.count = np.prod(shape)
            np_field_type = item_dtype
        else:
            pf.count = 1
        pf.datatype = nptype_to_pftype[np_field_type]
        pf.offset = field_offset
        fields.append(pf)
    return fields

def array_to_pointcloud2(cloud_arr, stamp=None, frame_id=None):
    '''Converts a numpy record array to a sensor_msgs.msg.PointCloud2.
    '''
    # make it 2d (even if height will be 1)
    # cloud_arr = np.atleast_2d(cloud_arr)

    # cloud_msg = PointCloud2()

    # if stamp is not None:
    #     cloud_msg.header.stamp = stamp
    # if frame_id is not None:
    #     cloud_msg.header.frame_id = frame_id
    
    # #cloud_msg.height = cloud_arr.shape[0]
    # #cloud_msg.width = cloud_arr.shape[1]
    # cloud_msg.height = 1
    # cloud_msg.width = cloud_arr.size
    # cloud_msg.fields.append(PointField(
    #                         name="x",
    #                         offset=0,
    #                         datatype=PointField.FLOAT32, count=1))
    # cloud_msg.fields.append(PointField(
    #                         name="y",
    #                         offset=4,
    #                         datatype=PointField.FLOAT32, count=1))
    # cloud_msg.fields.append(PointField(
    #                         name="z",
    #                         offset=8,
    #                         datatype=PointField.FLOAT32, count=1))
    
    # cloud_msg.is_bigendian = False # assumption
    # cloud_msg.point_step = 4

    # cloud_msg.row_step = cloud_msg.point_step*cloud_msg.width
    # cloud_msg.is_dense = False
    # cloud_msg.data = cloud_arr.tostring()
    # return cloud_msg 

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



def fields_to_dtype(fields, point_step):
    '''Convert a list of PointFields to a numpy record datatype.
    '''
    offset = 0
    np_dtype_list = []
    for f in fields:
        while offset < f.offset:
            # might be extra padding between fields
            np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
            offset += 1

        dtype = pftype_to_nptype[f.datatype]
        if f.count != 1:
            dtype = np.dtype((dtype, f.count))

        np_dtype_list.append((f.name, dtype))
        offset += pftype_sizes[f.datatype] * f.count

    # might be extra padding between points
    while offset < point_step:
        np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
        offset += 1
        
    return np_dtype_list

def pointcloud2_to_array(cloud_msg, squeeze=True):
    ''' Converts a rospy PointCloud2 message to a numpy recordarray 
    
    Reshapes the returned array to have shape (height, width), even if the height is 1.

    The reason for using np.fromstring rather than struct.unpack is speed... especially
    for large point clouds, this will be <much> faster.
    '''
    # construct a numpy record type equivalent to the point type of this cloud
    dtype_list = fields_to_dtype(cloud_msg.fields, cloud_msg.point_step)
    
    # parse the cloud into an array
    cloud_arr = np.fromstring(cloud_msg.data, dtype_list)

    # remove the dummy fields that were added
    cloud_arr = cloud_arr[
        [fname for fname, _type in dtype_list if not (fname[:len(DUMMY_FIELD_PREFIX)] == DUMMY_FIELD_PREFIX)]]
    
    if squeeze and cloud_msg.height == 1:
        return np.reshape(cloud_arr, (cloud_msg.width,))
    else:
        return np.reshape(cloud_arr, (cloud_msg.height, cloud_msg.width))
       
def convert_pc_msg_to_np(pc_msg):
    # Fix rosbag issues, see: https://github.com/eric-wieser/ros_numpy/issues/23
    pc_msg.__class__ = sensor_msgs.msg._PointCloud2.PointCloud2
    offset_sorted = {f.offset: f for f in pc_msg.fields}
    pc_msg.fields = [f for (_, f) in sorted(offset_sorted.items())]

    # Conversion from PointCloud2 msg to np array.
    pc_np = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc_msg, remove_nans=True)
    pc_pcl = pcl.PointCloud(np.array(pc_np, dtype=np.float32))
    return pc_np, pc_pcl  # point cloud in numpy and pcl format

def do_passthrough(pcl_data,filter_axis,axis_min,axis_max):
    '''
    Create a PassThrough  object and assigns a filter axis and range.
    :param pcl_data: point could data subscriber
    :param filter_axis: filter axis
    :param axis_min: Minimum  axis to the passthrough filter object
    :param axis_max: Maximum axis to the passthrough filter object
    :return: passthrough on point cloud
    '''
    passthrough = pcl_data.make_passthrough_filter()
    passthrough.set_filter_field_name(filter_axis)
    passthrough.set_filter_limits(axis_min, axis_max)
    return passthrough.filter()
#----------------------------------------------------------------------------------------------------------------------------------------


# liDAR data receiving class
class rpScanReceiver():
    
    def __init__(self):
        rospy.init_node('test', anonymous=True)
        
        self.sub = rospy.Subscriber('/livox/lidar', PointCloud2, self.callback)
        self.sub = rospy.Subscriber('/livox/imu', Imu, self.callbackImu)
        
        self.pub = rospy.Publisher("/out", PointCloud2, queue_size=1)
        rospy.spin()
    
    
    #downSampling function (Create a VoxelGrid filter object for a input point cloud)
    def do_voxel_grid_downssampling(self,pcl_data,leaf_size):
        vox = pcl_data.make_voxel_grid_filter()
        vox.set_leaf_size(leaf_size, leaf_size, leaf_size) # The bigger the leaf size the less information retained
        return  vox.filter()
    
    
    # euclidean clustering function (param cloud_objects) (return: cluster cloud and cluster indices)
    def do_euclidean_clustering(white_cloud):
        tree = white_cloud.make_kdtree()

        # Create Cluster-Mask Point Cloud to visualize each cluster separately
        ec = white_cloud.make_EuclideanClusterExtraction()
        ec.set_ClusterTolerance(0.015)
        ec.set_MinClusterSize(50)
        ec.set_MaxClusterSize(20000)
        ec.set_SearchMethod(tree)
        cluster_indices = ec.Extract()
        cluster_color = get_color_list(len(cluster_indices))

        color_cluster_point_list = []

        for j, indices in enumerate(cluster_indices):
            for i, indice in enumerate(indices):
                color_cluster_point_list.append([white_cloud[indice][0],
                                                white_cloud[indice][1],
                                                white_cloud[indice][2],
                                                rgb_to_float(cluster_color[j])])

        cluster_cloud = pcl.PointCloud_PointXYZRGB()
        cluster_cloud.from_list(color_cluster_point_list)
        return cluster_cloud,cluster_indices
    
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
    

    #  data processing function
    # def function1(self,pcl_data):
        
    #     for i in range(len(pcl_data.range())):
    #         x = pcl_data[0]
    #         y = pcl_data[1]
    #         z = pcl_data[2]
    #         dis = sqrt(x*x + y*y + z*z)
              
        
    #     return pcl_data

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
        
        
        #print(math.sqrt(abs(x)*abs(x) + abs(y)*abs(y) + abs(z)*abs(z)))
        #print(x, y, z)

        #angle vector g to axis
        theta_x = math.acos(x/g)
        theta_y = math.acos(y/g)
        theta_z = math.acos(z/g)

        #theta rotate by y axis
        theta_y_trans = math.acos(-z/math.sqrt(x*x + z*z))
        theta_x_trans = -1 * math.acos(math.sqrt(x*x+z*z)/g)

        #print("y_axis rotation : ", theta_y_trans * 57.29577951308232, " // x_axis rotation : ", theta_x_trans * 57.29577951308232)
    
        # print(x, y, z, sep='   ///   ')
        


    def callback(self, input_ros_msg):
        cloud = ros_to_pcl(input_ros_msg)
        
        #cloud_arr, cloud = convert_pc_msg_to_np(input_ros_msg)
        num = cloud.size
        print("before downsampling:", cloud)
        
        ##  --------------------------------------- downsampling  -------------------------##
        LEAF_SIZE = 0.2
        cloud = self.do_voxel_grid_downssampling(cloud, LEAF_SIZE)
        print("after downsampling:", cloud)
        print("")

        ##  -----------------Statistical Outlier Removal ---------------##
        filter_axis = 'x'
        axis_min = 0.0
        axis_max = 100.0
        cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)

        filter_axis = 'y'
        axis_min = -10.0
        axis_max = 10.0
        cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)

        filter_axis = 'z'
        axis_min = -2.0
        axis_max = 2.0
        cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)
        
        ##  ----------------- Statistical Outlier Removal ---------------##
        
        cloud = self.do_passthrough(cloud, 'x', 1.0, 20.0)
        cloud = self.do_passthrough(cloud, 'y', -7.0, 5.5)
        _, floor, cloud = self.do_ransac_plane_normal_segmentation(cloud, 0.05)
        
        #cloud = floor
        cloud_arr = cloud.to_array()
        
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
            #print(x, z, cloud_arr[point][0], cloud_arr[point][1])
        #print("point 1 - y axis rotate : ", cloud_arr[1])
        
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
            print("2 is not empty")
        else:
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
        
        
        # Euclidean Clustering

        # white_cloud= XYZRGB_to_XYZ(cloud)
        # cluster_cloud,cluster_indices = self.do_euclidean_clustering(white_cloud)
        # print("after clustering:", cluster_cloud)
        # print("")
        
        # pcl.save(cluster_cloud,"livox_horizon.pcd")
        
        
        
        
        #out_msg = PointCloud2()
        #PCL을 ROS 메시지로 변경
        #out_msg = pcl_to_ros(cloud)
        #out_msg = array_to_pointcloud2(cloud_arr, stamp=rospy.Time.now(), frame_id='livox_output')   
        tmp_msg = PointCloud2()
        #tmp_msg = pcl_to_ros(floor)
        tmp_msg = array_to_pointcloud2(cloud_arr, rospy.Time.now(), frame_id='output')
        #tmp_arr = ros_to_pcl(tmp_msg)
        self.pub.publish(tmp_msg)

#------------------------------main---------------------------------
if __name__=="__main__":
    
    rp = rpScanReceiver()