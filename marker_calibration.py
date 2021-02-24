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
		self.del_pixel_static = [abs(x - y) for x, y in zip(self.static_marker.corner1,  self.static_marker.corner2)]
		self.del_pixel_dynamic = [abs(x - y) for x, y in zip(self.dynamic_marker.corner1, self.dynamic_marker.corner2)]


		#We already measured real distance between markers in mm
		self.depth_resolution = self.delta_depth/abs(self.del_pixel_static - self.del_pixel_dynamic) #mm/pixel



	def compute_width_resolution(self):
		"""Using a the static marker width, we can get the pixel-to-world mapping"""

		self.width_resolution = self.static_marker.width/self.del_pixel_static\

	def save_to_file(self):
		file_path = "" # Add later

		res_array = np.array([self.width_resolution, self.depth_resolution])

		np.save(file_path, res_array)





class ImageProcessing():
	def __init__(self, camera_topic):

		# Create marker objects for each marker type
		self.static_marker = Marker(0, "static")
		self.dynamic_marker = Marke(5, "dynamic")

		self.image_sub = rospy.Subscriber(camera_topic, Image, self.calibrate)

		# Get camera matrices
		self.mtx = np.load("put file_path")
		self.dist = np.load("put file_path")

		# Initialize bridge
		self.bridge = CvBridge()


	def calibrate(self, img_msg):

		# Define aruco library
		aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
		parameters = aruco.DetectorParameters_create()

		# Get image messages
		try:
			cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
		except CvBridgeError as e:
			print(e)


		# Convert to gray scale
		gray_side = cv2.cvtColor(cv_image_side, cv2.COLOR_BGR2GRAY)

		# Find markers in the image
		corners, ids, rejected = aruco.detectMarkers(image=gray_side, dictionary=aruco_dict, parameters=parameters, cameraMatrix=self.mtx, distCoeff=self.dist)

		# corners, ids, rejected = aruco.detectMarkers(image=gray_side, dictionary=aruco_dict, parameters=parameters)
		
		if np.all(ids != None):

			rvec, tvec, _objPoints = aruco.estimatePoseSingleMarkers(corners,self.marker_size, self.mtx, self.dist)

			for i in range(ids.size):
				#Draw reference frame for the marker
				if ids[i] == self.static_marker.id:
					corners = corners[i][0]
					self.static_marker.update_marker_corners(corners)

				if ids[i] == self.dynamic_marker.id:
					corners = corners[i][0]


			aruco.drawDetectedMarkers(cv_image, corners, ids)

			# Draw stuff here


		# Display
		cv2.imshow('frame', cv_image_side)
		cv2.waitKey(3)

	def pixel2world(self):
		"""Return the tracking resolution and draws markers"""
		pass


	def draw(self):
		"""Draw relative distance between """



def main(args):
	ImageProcessing()
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