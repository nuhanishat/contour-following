#!/usr/bin/env python

#Author: Nuha Nishat
#Date: 10/12/20

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
import numpy as np 
import cv2.aruco as aruco
import math
import sys
from transforms import *

class Path_Generator():
	def __init__(self):
		self.path_pub = rospy.Publisher('/arm_control/path_generator/planned_path', Float32, queue_size=10)
		self.bridge = CvBridge()
		self.flag_pub = rospy.Publisher('/arm_control/path_generator/feedback', String, queue_size=10)
		# rospy.init_node('path_generator', anonymous=True)

		# Aruco Marker Info
		self.marker_size = 7 #cm

		# Marker IDs
		self.robot_marker_id_side = 0
		self.obj_marker_id_side = 5
		# self.robot_marker_id_top = 14
		# self.obj_marker_id_top = 9

		# Get the saved camera and distortion matrices from caliberation
		self.mtx = np.load("/home/nuha/kinova_ws/src/arm_control/src/CameraMatrices/camera_mtx.npy")
		self.dist = np.load("/home/nuha/kinova_ws/src/arm_control/src/CameraMatrices/dist_mtx.npy")

		# This is the imahe message subscriber. Chnage the topic to your camera topic
		self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.get_mold_pose)

	def get_mold_pose(self, img_msg):

		# Define Aruco Dictionary 
		aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
		parameters = aruco.DetectorParameters_create()

		# List for storing marker positions 
		robot_marker_side = []
		obj_marker_side = []
		robot_marker_top = []
		obj_marker_side = []

		# These are the image messages. There are two camera topics
		
		try:
			cv_image_side =self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
		except CvBridgeError as e:
			print(e)

		
		# Read camera 
		# cv_image_top = self.bridge.imgmsg_to_cv2(cv_image_top, "brg8")
		# cv_image_side = self.bridge.imgmsg_to_cv2(img_side, "brg8")

		# Convert in gray scale
		# gray_top = cv2.cvtColor(cv_image_side, cv2.COLOR_BGR2GRAY)

		# cv_image_side = cv2.imread("/home/nuha/kinova_ws/src/arm_control/src/CameraMatrices/test_markers1.jpeg",0)

		# Convert to gray scale
		gray_side = cv2.cvtColor(cv_image_side, cv2.COLOR_BGR2GRAY)

		# Find markers in the image
		corners, ids, rejected = aruco.detectMarkers(image=gray_side, dictionary=aruco_dict, parameters=parameters, cameraMatrix=self.mtx, distCoeff=self.dist)

		# corners, ids, rejected = aruco.detectMarkers(image=gray_side, dictionary=aruco_dict, parameters=parameters)
		
		if np.all(ids != None):

			rvec, tvec, _objPoints = aruco.estimatePoseSingleMarkers(corners,self.marker_size, self.mtx, self.dist)

			for i in range(ids.size):
				#Draw reference frame for the marker
				if ids[i] == self.robot_marker_id_side:
					robot_marker_side = corners[i][0]

				if ids[i] == self.obj_marker_id_side:
					obj_marker_side = corners[i][0]

				# Draw Axis
				# aruco.drawAxis(cv_image_side, self.mtx, self.dist, rvec[i], tvec[i], 7)
			# 	print("No Markers Found")
			#Draw Markers
			aruco.drawDetectedMarkers(cv_image_side, corners, ids)

			# Find the tracking resolution 
			self.tracking_res = self.marker_caliberation(robot_marker_side)

			# view = 'side'
			view = 'top'

			self.draw_marker(robot_marker_side, 'robot_base', view, cv_image_side)
			self.draw_marker(obj_marker_side, 'mold', view , cv_image_side)
			# self.draw_relative_pose(robot_marker_side, obj_marker_side, cv_image_side)

			path = create_path(21, 14, 14)
			print(path)

			# self.draw_path(view,path,obj_marker_side,cv_image_side)

		# Display
		cv2.imshow('frame', cv_image_side)
		cv2.waitKey(3)

	def marker_caliberation(self, marker_corners):
		'''This function does the initial caliberation 
		given the position of the camera. This is used for the reference marker at the robot base.
		Returns the tracking resolution'''
		robot_marker_side_corner1 = marker_corners[0]
		robot_marker_side_corner2 = marker_corners[1]

		tracking_res = find_tracking_resolution(self.marker_size, [robot_marker_side_corner1[0], robot_marker_side_corner2[0]])


		# Rotate points if angle is larger than 5 degrees
		# angle, axis = get_marker_rotation(tracking_res,[robot_marker_side_corner1,robot_marker_side_corner2], self.marker_size)

		# if angle > 5*np.pi/180:
			# new_robot_marker_side_corners = rotate_point(robot_marker_side_corner1, robot_marker_side_corner2, angle)


		return tracking_res

	def draw_marker(self,marker_corners,marker_body,view,cv_image):
		'''Draw all the important stuff for each marker'''

		image_width = 1920
		image_height = 1080


		if view == 'side': 
			marker_side_corner1 = marker_corners[0]
			marker_side_corner2 = marker_corners[1]
			marker_side_corner3 = marker_corners[2]
			marker_side_corner4 = marker_corners[3]

			marker_side_center = get_marker_center(marker_side_corner1, marker_side_corner2, marker_side_corner3, marker_side_corner4)

			# print('center: ', marker_side_center)

			if marker_body == 'robot_base':

				cv2.circle(cv_image,(marker_side_corner4[0],marker_side_corner4[1]), radius=10, color=(0,0,255), thickness=-1)

				cv2.putText(cv_image,'Robot Base', (marker_side_center[0]-10, marker_side_center[1]-10), cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,0),2,cv2.LINE_AA)

			elif marker_body == 'mold':

				cv2.circle(cv_image,(marker_side_corner4[0],marker_side_corner4[1]), radius=10, color=(0,0,255), thickness=-1)

				cv2.putText(cv_image,'End-effector', (marker_side_center[0]-10, marker_side_center[1]-10), cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,0),2,cv2.LINE_AA)

			cv2.line(cv_image, (100,50),(100,150),(255,255,0),3)
			cv2.putText(cv_image, 'Z',(100,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,0),2,cv2.LINE_AA)
			cv2.line(cv_image, (100,150),(250,150),(255,255,0),3)
			cv2.putText(cv_image, 'Y',(250,160),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,0),2,cv2.LINE_AA)

			# Add text to view
			cv2.putText(cv_image, 'Side View',(1920/2,100),cv2.FONT_HERSHEY_SIMPLEX,3,(255,255,0),3,cv2.LINE_AA)
		
		elif view == 'top':
			marker_top_corner1 = marker_corners[0]
			marker_top_corner2 = marker_corners[1]
			marker_top_corner3 = marker_corners[2]
			marker_top_corner4 = marker_corners[3]

			marker_top_center = marker_side_center = get_marker_center(marker_top_corner1, marker_top_corner2, marker_top_corner3, marker_top_corner4)


			if marker_body == 'robot_base' :
				# y offset
				y_offset = (7/2.0)*(1/float(self.tracking_res))

				print(int(y_offset))
				print(marker_top_corner3[1])
				
				# Draw the origin of robot base
				# cv2.circle(cv_image,(marker_top_corner2[0], int(marker_top_corner2[1] - y_offset)),radius=10, color=(0,0,255), thickness=-1)

				cv2.circle(cv_image,(marker_top_center[0], int(marker_top_center[1])),radius=10, color=(0,0,255), thickness=-1)
				cv2.putText(cv_image,'Robot Base', (marker_top_center[0]-10, marker_top_center[1]-10), cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,0),2,cv2.LINE_AA)

			elif marker_body == 'mold':

				# Draw the origin of Mold
				y_offset = (7/2.0)*(1/float(self.tracking_res))

				# cv2.circle(cv_image,(marker_top_corner4[0], int(marker_top_corner4[1] - y_offset)),radius=10, color=(0,0,255), thickness=-1)

				# cv2.circle(cv_image,(marker_top_corner2[0],marker_top_corner2[1]), radius=10, color=(0,0,255), thickness=-1)

				# cv2.circle(cv_image,(marker_top_center[0],marker_top_center[1]), radius=10, color=(0,0,255), thickness=-1)
				cv2.putText(cv_image,'End-effector', (marker_top_center[0]-10, marker_top_center[1]-10), cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,0),2,cv2.LINE_AA)

				# Draw the angle of rotation of mold marker

				angle, axis = get_marker_rotation([marker_top_corner1, marker_top_corner2], self.marker_size)

				cv2.line(cv_image,(marker_top_corner1[0], marker_top_corner1[1]), (marker_top_corner2[0], marker_top_corner2[1]),(0,255,0),5)

				angle_deg = 180*angle/np.pi

				cv2.line(cv_image, (marker_top_corner1[0], marker_top_corner1[1]), (int(axis[0]), int(axis[1])), (255, 0, 0), 5)

				cv2.putText(cv_image, str(int(angle_deg)) + 'deg',(marker_top_corner1[0], marker_top_corner1[1]),cv2.FONT_HERSHEY_SIMPLEX,3,(255,255,255),3)

			# Draw the axes on image
			cv2.line(cv_image, (100,50),(100,150),(255,255,0),3)
			cv2.putText(cv_image, 'X',(100,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,0),2,cv2.LINE_AA)

			cv2.line(cv_image, (100,150),(250,150),(255,255,0),3)
			cv2.putText(cv_image, 'Y',(250,160),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,0),2,cv2.LINE_AA)

			# Add text to view
			cv2.putText(cv_image, 'Top View',(1920/2,100),cv2.FONT_HERSHEY_SIMPLEX,3,(255,255,0),3,cv2.LINE_AA)


	def draw_relative_pose(self, marker_corners_base, marker_corners_ee, cv_image):
		marker_center_base = get_marker_center(marker_corners_base[0], marker_corners_base[1], marker_corners_base[2], marker_corners_base[3])
		marker_center_ee = get_marker_center(marker_corners_ee[0], marker_corners_ee[1], marker_corners_ee[2], marker_corners_ee[3])

		difference = [(i - j)*self.tracking_res for i, j in zip(marker_center_ee, marker_center_base)]

		cv2.putText(cv_image,'('+str(difference[0])+', '+ str(difference[1]) + ') cm', (marker_center_ee[0], marker_center_ee[1] + 15), cv2.FONT_HERSHEY_SIMPLEX,1,(25,255,0),2,cv2.LINE_AA)


	# def create_path(self,mold_length,mold_width,mold_height):
	# 	'''All units are in cm for now'''

	# 	# x_points = [for i range()]

	# 	x_offset = 0 # Direction  of movement
	# 	y_offset = (mold_width/2.0)*self.tracking_res # Need to check x and y axes on arm 
	# 	z_offset = mold_height*self.tracking_res

	# 	n = 20
	# 	res = (mold_length/n)

	# 	path = []

	# 	y = [res*i for i in range(n+1)]

	# 	z = [math.tanh(x) for x in range(n+1) ]

	# 	for i in range(6):
	# 		path.append([x_offset+res*i, y_offset, z_offset])

	# 	return path

	def draw_path(self, view, path, mold_marker_corners, cv_image):

		image_width = 1920
		image_height = 1080
		
		marker_corner1 = mold_marker_corners[0]
		marker_corner2 = mold_marker_corners[1]
		marker_corner3 = mold_marker_corners[2]
		marker_corner4 = mold_marker_corners[3]


		if view == 'side':
			y_path = [(path[i][1]-7)*(1.0/self.tracking_res) for i in range(len(path))]
			z_path = [path[i][2]*(1.0/self.tracking_res)for i in range(len(path))]

			print(y_path)
			print(z_path)
			
			for i in range(len(y_path)-1):
				cv2.line(cv_image, (int(marker_corner4[0]+ y_path[i]), int(marker_corner4[1] - z_path[i])),(int(marker_corner4[0]+ y_path[i+1]), int(marker_corner4[1] - z_path[i+1])),(255,0,0),3)

		elif view == 'top':
			pass




def main(args):
	f = Path_Generator()
	rospy.init_node('path_generator', anonymous=True)

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting Down")
	cv2.destroyAllWindows()



if __name__ == '__main__':
	main(sys.argv)




