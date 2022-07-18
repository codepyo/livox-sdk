#!/usr/bin/env python

from curses import color_content
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

def float_to_rgb(float_rgb):
    
    s = struct.pack('>f', float_rgb)
    i = struct.unpack('>l', s)[0]
    pack = ctypes.c_uint32(i).value
			
    r = (pack & 0x00FF0000) >> 16
    g = (pack & 0x0000FF00) >> 8
    b = (pack & 0x000000FF)
			
    color = [r,g,b]
			
    return color

# axis_min & axis_max : minimum or maximum axis to passsthrough filter object

def do_passthrough(pcl_data,filter_axis,axis_min,axis_max):
    
    passthrough = pcl_data.make_passthrough_filter()
    passthrough.set_filter_field_name(filter_axis)
    passthrough.set_filter_limits(axis_min, axis_max)
    return passthrough.filter()

def do_ransac_plane_normal_segmentation(point_cloud, input_max_distance):
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


#-----------------------------------------------------------------------------------
test1 = o3d.io.read_point_cloud("lanedetection.pcd")
o3d.visualization.draw_geometries([test1])

#-----------------------------------------------------------------------------------

test1 = o3d.io.read_point_cloud("livox_horizon.pcd")
o3d.visualization.draw_geometries([test1])

cloud = pcl.load("livox_horizon.pcd")
# for i in range(0, 20):#pc.size):
#     print ('x: ' + str(cloud[i][0]) + ', y : ' + str(cloud[i][1]) + ', z : ' + str(cloud[i][2]) + ' , rgb : ' + str(cloud[i][3]))

cloud = do_passthrough(cloud, 'x', 1.0, 20.0)
cloud = do_passthrough(cloud, 'y', -7.0, 5.5)
_, floor, cloud = do_ransac_plane_normal_segmentation(cloud, 0.05)

pcl.save(cloud,"livox_horizon_nofloor.pcd")
test1 = o3d.io.read_point_cloud("livox_horizon_nofloor.pcd")


# for i in range(0, 20):#pc.size):
#     print ('x: ' + str(cloud[i][0]) + ', y : ' + str(cloud[i][1]) + ', z : ' + str(cloud[i][2]) + ' , rgb :  : ' + str(cloud[i][3]))
o3d.visualization.draw_geometries([test1])

pcl.save(floor,"livox_horizon_floor.pcd")

# color_list=[]
# color_list = float_to_rgb(floor[3])

for i in range(0, 20):#pc.size):
    print ('x: ' + str(floor[i][0]) + ', y : ' + str(floor[i][1]) + ', z : ' + str(floor[i][2]) )


# for i in range(floor.size):
#     color_list.append(float_to_rgb(floor[3]))
    

# for i in range(0, 20):#pc.size):
#     print ('r: ' + str(color_list[i][0]) + ', g : ' + str(color_list[i][1]) + ', b : ' + str(color_list[i][2]))

test1 = o3d.io.read_point_cloud("livox_horizon_floor.pcd")
o3d.visualization.draw_geometries([test1])

# mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
# mesh_r = copy.deepcopy(test1)
# R = mesh.get_rotation_matrix_from_xyz((np.pi / 2, 0, np.pi / 4))
# mesh_r.rotate(R, center=(0, 0, 0))

# o3d.visualization.draw_geometries([mesh_r])
#o3d.visualization.draw_geometries([test1, mesh_r])


# for i in range(len(pc.range())):
#     sdefs


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




