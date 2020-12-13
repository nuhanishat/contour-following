#!/usr/bin/env python

#Author: Nuha Nishat
#Date: 10/14/20


import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
# from sensor_msgs.msg import Image
# from std_msgs.msg import String, Float32
import numpy as np 
import cv2.aruco as aruco
import math
import sys
import numpy as np


def translate_point(tvec):
	pass

def get_marker_center(corner1,corner2,corner3,corner4):

	center_x = corner1[0] + (corner2[0] - corner1[0])/2.0

	center_y = corner1[1] - (corner2[1] - corner3[1])/2.0
	
	return [int(center_x), int(center_y)]

def rotate_point(point1, point2, angle):
	'''Takes in two points and angle of rotation to return a '''
	
	# Rotation Matrix
	R = np.identity(3)
	R[0][0] = np.cos(angle)
	R[0][1] = -np.sin(angle)
	R[1][0] = np.sin(angle)
	R[1][1] = np.cos(angle)

	# Translation Matrix
	T = np.identity(3)
	T[0,2] = point2[0]-point1[0]
	T[1,2] = point2[1]-point1[1]

	mat = np.matmul(T,R)

	points = np.transpose(np.array([[point1, point2, 1]]))

	new_points = np.matmul(mat, points)

	new_points = np.transpose(new_points)

	print(new_points)


def get_marker_rotation(adj_corners,marker_size):
	'''Takes in the tracking resolution and the top two marker corners as input to estimate 
	a marker angle'''

	corner1 = adj_corners[0]
	corner2 = adj_corners[1]

	# This is the corner position that corner2 should be at when not rotated
	corner2_temp_x = corner1[0] + 210

	corner2_temp_y = corner1[1]


	# For drawing the reference line 
	axis = [corner2_temp_x, corner2_temp_y]

	print('corner1: ', corner1)
	print('corner2: ', corner2)

	angle = math.atan((corner2[1] - corner1[1])/(corner2[0] - corner1[0]))


	#First Quadrant
	if corner2[0] >= corner1[0] and corner1[1] >= corner2[1]:
		angle *= -1

	# Second Quadrant
	if corner1[0] > corner2[0] and corner1[1] > corner2[1]:
		angle -= np.pi
		angle *= -1

	# Third Quadrant
	if corner1[0] >= corner2[0] and corner1[1] <= corner2[1]:
		angle += np.pi
		angle *= -1

	# Fourth Quadrant
	if corner1[0] < corner2[0] and corner1[1] < corner2[1]:
		angle *= -1

	return angle, axis

def draw_path(path):
	'''Path is a list of trajectory point'''
	pass

def create_path(mold_length,mold_width,mold_height):
	'''All units are in cm for now'''

	# x_points = [for i range()]

	x_offset = 0 # Direction  of movement
	y_offset = (mold_width/2.0) # Need to check x and y axes on arm 
	z_offset = mold_height

	n = 20
	res = (mold_length/float(n))

	path = []

	y = [res*i for i in range(n+1)]

	z = [math.sin(300*x) for x in range(n+1)]

	x = [ x*0 for x in range(n+1)]

	for i in range(n+1):
		path.append([x_offset+x[i], y_offset+y[i], z_offset+z[i]])

	return path

# This should be used only with the robot base marker which is fixed relative to the camera
def map_pixel_to_real_world_position(tracking_res, start_pixel, end_pixel):
	'''Given the tracking resolution, map pixel position to real world'''
	start_pos = start_pixel*tracking_res
	end_pos = start_pos + (end_pixel - start_pixel)*tracking_res
	return end_pos

def map_real_world_position_to_pixel(tracking_res, start_position, end_position):
	'''Given the tracking resolution, map real world position to pixels'''
	start_pixel = start_position * (1.0/float(tracking_res))
	end_pixel = start_pixel + (end_position - start_position)*(1.0/float(tracking_res))
	return end_pixel

# This will change with the size of marker workspace, try to place camera as close as possible
def find_tracking_resolution(marker_size, adj_corners_x):
	tracking_res = (adj_corners_x[1] - adj_corners_x[0])/marker_size # In pixel/cm
	return 1.0/float(tracking_res) #In cm/pixel


# def map_x_to_pixel(tracking_res, ):


if __name__ == '__main__':

	# #Load an image 
	# img = cv2.imread("/home/nuhanishat/new_kinova_ws/src/kinova-ros/arm_control/src/test_markers1.jpg",0)

	# path = create_path(0.8, 0.4, 0.2)
	# print(path)

	# # Show image
	# cv2.imshow('image',img)
	# cv2.waitKey(0)
	# cv2.destroyAllWindows()

	# res = find_tracking_resolution(7, [316, 522])

	# print(str(res*10) + ' mm/pixel')

	path = create_path(14, 12, 10)

	print(path)




