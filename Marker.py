#!/usr/bin/env python

#Author: Nuha Nishat
#Date: 10/12/20

# import rospy
import numpy as np 
import math

class Marker():
	"""Marker object for saving the different marker types and corresponding
	parameters
	"""
	def __init__(self, marker_id, marker_type):
		self.id = marker_id
		self.type = marker_type
		
		# Placeholders the marker pose in the real world
		self.x = None
		self.y = None
		self.theta = 0

		# Pixel position
		self.px = None
		self.py = None

		# Place holders for marker corners
		self.corner1 = []
		self.corner2 = []
		self.corner3 = []
		self.corner4 = []

		# Save the tracking resolution
		self.resolution = None

		# Saving marker width here
		self.width = 60 #mm

	def update_marker_pose(self, x, y, z, theta):
		"""Update the marker pose"""
		self.x = x
		self.y = y
		self.z = z
		self.theta = theta

	def marker_pose(self):
		# Save pose 
		pose = (x, y, z, theta)
		return pose

	def update_marker_corners(self, corners):
		""" Save marker corners
			:param: corners - list of marker corners
		"""
		self.corner1 = corners[0]
		self.corner2 = corners[1]
		self.corner3 = corners[2]
		self.corner4 = corners[3]


	






