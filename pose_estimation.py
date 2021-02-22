#!/usr/bin/env python

#Author: Nuha Nishat
#Date: 2/21/21

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
import numpy as np 
import cv2.aruco as aruco
import math
import sys
import transforms 

class PoseEstimation():
	def __init__(self, marker_size = 60, marker_ids, camera_topic):
		"""Detects marker corners and estimate the marker pose

			:param: marker size in mm
			:param: marker_ids - list of marker ids
			:param: camera_topic - the camera topic name - string
		"""

		self.bridge = CvBridge()

		self.marker_size = marker_size

		self.marker_ids = marker_ids

		# Get the saved camera and distortion matrices from caliberation
		self.mtx = np.load("/home/mechagodzilla/kinova_ws/src/contour-following/src/CameraMatrices/camera_mtx.npy")
		self.dist = np.load("/home/mechagodzilla/kinova_ws/src/contour-following/src/CameraMatrices/dist_mtx.npy")

		# This is the image message subscriber. Chnage the topic to your camera topic
		self.image_sub = rospy.Subscriber(camera_topic, Image, self.get_pose)

		# Define the Aruco Dictionary 
		self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
		self.parameters = aruco.DetectorParameters_create()


	def compute_marker_pose(self, marker):
		"""Get the marker center
		:param: Marker object
		"""

		# Marker center in marker frame 
		local_center_x = (marker.corner2[0] - marker.corner1[0])/2.0
		local_center_y = - (marker.corner2[1] - marker.corner3[1])/2.0

		# Calculate the angle of marker in global camera frame
		delta_corners = [x - y for x, y in zip(self.corner1, self.corner2)]
		
		if abs(delta_corners[1]) > 0: # if there is difference in y coord
			angle = math.atan(delta_corners[1]/delta_corners[0])
			marker.theta = angle # Update pose angle by this angle
		
		# Rotate center by angle and translate by corner 1
		marker.x = marker.corner1[0] + (local_center_x*np.cos(marker.theta) - local_center_y*np.sin(marker.theta))
		marker.y = marker.corner1[1] + (local_center_x*np.sin(marker.theta) + local_center_y*np.cos(marker.theta))


	def pixel_to_world(self, marker):
		




	def caliberate(self, marker_corners):
		"""Gives the tracking resolution for marker
		"""
		pass























if __name__ == '__main__':
