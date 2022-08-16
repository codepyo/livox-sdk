
import numpy as np  
import cv2 
import matplotlib.image as mpimg
from matplotlib import pyplot as plt 
import open3d
from scipy import signal
import argparse
import copy

# ap = argparse.ArgumentParser()
# ap.add_argument("-i", "--image", type = str, required=True, help="the path to the input image")
# ap.add_argument("-d", "--distort", type = int, required=True, help="1 means distort the image point cloud")
# args = vars(ap.parse_args())
# path_image = args["image"]
# path_image = cv2.imread('parkpic3.jpg')
distort = 0

#Preprocess the image, resize, add noise if required
# img = mpimg.imread(path_image)
img9 = mpimg.imread('parkpic7.jpg')
l , w, c  = img9.shape

lower = np.array([220, 220, 220])
upper = np.array([255, 255, 255])

thresh = cv2.inRange(img9, lower, upper)
img9 = cv2.imwrite('parkpic6_thresh2.jpg', thresh)
img9 = cv2.imread('parkpic6_thresh2.jpg')

img = cv2.resize(img9,(256,256))
# img = cv2.resize(img,(l//20,w//20))
l , w, c  = img.shape
print(l,w,c)

""" add noise
mu =  np.max(img) / 25
print("mu is :", mu)
noise = np.random.normal(0,mu,img.shape)
img = noise + img
"""

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
	
	if distort == 1:
	     my_img_position = distort_pcd(my_img_position)
	
	my_img_color = np.concatenate((intensity_f_r,intensity_f_g,intensity_f_b),axis=1)
	my_img_color = ((my_img_color - my_img_color.min()) / (my_img_color.max() - my_img_color.min()))
 
 
	pcd = open3d.geometry.PointCloud()
	pcd.points = open3d.utility.Vector3dVector(my_img_position)
	pcd.colors = open3d.utility.Vector3dVector(my_img_color)
 
	# mesh = open3d.geometry.TriangleMesh.create_coordinate_frame()
	# mesh_r = copy.deepcopy(mesh)
	# R = mesh.get_rotation_matrix_from_xyz((0, 0 ,np.pi))
	# mesh.rotate(R)
	# open3d.visualization.draw_geometries([mesh, mesh_r])
	# pcd.rotate(R) 
	open3d.io.write_point_cloud("image_pcd.ply", pcd)
	pcd_load = open3d.io.read_point_cloud("image_pcd.ply")
	open3d.visualization.draw_geometries([pcd_load])
	return my_img_position, my_img_color

my_img_position,  my_img_color = convert_imgTo_pcd(img)
np.savez('position.npz', position=my_img_position)
np.savez('texture.npz', texture=my_img_color)