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
import numpy as np
from Marker import *
from pose_estimation import *


class MarkerCalibration():
	"""Does marker caliberation of find depth in real world from marker size"""
	def __init__(self, static_marker, dynamic_marker, delta_depth):
		self.static_marker = static_marker
		self.dynamic_marker = dynamic_marker
		self.delta_depth = delta_depth

		## Call each method here 
		self.compute_depth_resolution()
		self.compute_width_resolution()

		# Save to file
		self.save_to_file()
	

	def compute_depth_resolution(self):
		"""Using similar triangles, calculates the depth"""

		# Calculate the pixel width
		self.del_pixel_static = max([abs(x - y) for x, y in zip(self.static_marker.corner1,  self.static_marker.corner2)])
		self.del_pixel_dynamic = max([abs(x - y) for x, y in zip(self.dynamic_marker.corner1, self.dynamic_marker.corner2)])


		#We already measured real distance between markers in mm
		self.depth_resolution = self.delta_depth/abs(self.del_pixel_static - self.del_pixel_dynamic) #mm/pixel



	def compute_width_resolution(self):
		"""Using a the static marker width, we can get the pixel-to-world mapping"""

		self.width_resolution = self.static_marker.width/self.del_pixel_static

	def save_to_file(self):
		file_path = "/home/mechagodzilla/kinova_ws/src/contour-following/src/CameraMatrices/resolution.npy" # Add later

		res_array = np.array([self.width_resolution, self.depth_resolution])

		np.save(file_path, res_array)





class ImageProcessing():
	def __init__(self, camera_topic):

		# Create marker objects for each marker type
		self.static_marker = Marker(0, "static")
		self.dynamic_marker = Marker(20, "dynamic")

		self.image_sub = rospy.Subscriber(camera_topic, Image, self.calibrate)

		# Get camera matrices
		self.mtx = np.load("/home/mechagodzilla/kinova_ws/src/contour-following/src/CameraMatrices/camera_mtx.npy")
		self.dist = np.load("/home/mechagodzilla/kinova_ws/src/contour-following/src/CameraMatrices/dist_mtx.npy")

		# Initialize bridge
		self.bridge = CvBridge()



	def calibrate(self, img_msg):

		# Define aruco library
		aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
		parameters = aruco.DetectorParameters_create()

		# Get image messages
		try:
			self.cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
		except CvBridgeError as e:
			print(e)

		x = 450
		y = 50
		h = 1000
		w = 1300



		self.cv_image = self.cv_image[y:y+h, x:x+w]


		# Convert to gray scale
		gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)

		# Find markers in the image
		corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters)

		# corners, ids, rejected = aruco.detectMarkers(image=gray_side, dictionary=aruco_dict, parameters=parameters)
		
		if np.all(ids != None):

			ret = aruco.estimatePoseSingleMarkers(corners,60, self.mtx, self.dist)

			(rvec, tvec) = (ret[0][0, 0, :], ret[1][0, 0, :])

			for i in range(ids.size):
				#Draw reference frame for the marker
				if ids[i] == self.static_marker.id:
					new_corners = corners[i][0]
					self.static_marker.update_marker_corners(new_corners)
					aruco.drawAxis(self.cv_image, self.mtx, self.dist, rvec, tvec, 7)
					# rospy.loginfo(str(self.static_marker.corner1))

				if ids[i] == self.dynamic_marker.id:
					new_corners = corners[i][0]
					self.dynamic_marker.update_marker_corners(new_corners)
					# print(self.static_marker.corner1)

			# aruco.drawAxis(self.cv_image, self.mtx, self.dist, rvec, tvec, 7)
			
			aruco.drawDetectedMarkers(self.cv_image, corners, ids)

			# Draw stuff here

			# Marker Calibration 
			cal = MarkerCalibration(self.static_marker, self.dynamic_marker, 381)
			self.pixel2world()





		# Display
		cv2.imshow('frame', self.cv_image)
		cv2.waitKey(3)

	def pixel2world(self):
		"""Return the tracking resolution and draws markers"""
		
		# Get the resolutions

		resolution = np.load("/home/mechagodzilla/kinova_ws/src/contour-following/src/CameraMatrices/resolution.npy")

		# Intitialise the pose estimation class
		pose_est = PoseEstimation()

		pose_est.compute_marker_pose(self.static_marker)
		pose_est.pixel_to_world(self.static_marker, resolution[0], resolution[1])
		pose_est.compute_marker_pose(self.dynamic_marker)
		pose_est.pixel_to_world(self.dynamic_marker, resolution[0], resolution[1])

		self.draw(self.static_marker, self.cv_image)
		self.draw(self.dynamic_marker, self.cv_image)

		


	def draw(self, marker, cv_image):
		"""Draw relative distance between """
		image_width = 1920
		image_height = 1080

		cv2.circle(cv_image,(int(marker.px), int(marker.py)), radius=10, color=(0,0,255), thickness=-1)

		cv2.putText(cv_image, marker.type, (int(marker.px), int(marker.py)), cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,0),2,cv2.LINE_AA)
		cv2.putText(cv_image, str((round(marker.x), round(marker.y), round(marker.z))), (int(marker.px), int(marker.py - 10)), cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,0),2,cv2.LINE_AA)
		


def main(args):
	ImageProcessing("/side_camera/usb_cam2/image_raw")
	rospy.init_node('calibration', anonymous=True)

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting Down")
	cv2.destroyAllWindows()



if __name__ == '__main__':
	main(sys.argv)













if __name__ == '__main__':
	# params = MarkerCalibration.get_params()

	## Run three times in 3 different depths and take the average


	"""
	- Pass in real-world delta depth of the two markers
	- Pass in the marker objects for each marker
	- Get the depth-to-pixel ratio from relative marker size difference for
	  calculating z
	- You can also calculate the size-to-pixel ratio for x, y. Do that here and save these
	ratios in a file"""