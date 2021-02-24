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
from Marker import *


class SideVision():
	def __init__(self):
		
		#Create Marker objects
		self.end_effector = Marker(30, "end_effector")
		self.workpiece = Marker(20, "workpiece")
		self.robot_base = Marker(10, "")







if __name__ == '__main__':
	
