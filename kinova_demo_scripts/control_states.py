#! usr/bin/env python

import rospy
import sys
from smach import State, StateMachine, CBState
from smach_ros import SimpleActionState, ServiceState
import math
from param_saver import *


class Start(State):
	'''State: Load the path file data as a ros parameter dictionary
	'''
	def __init__(self):
		State.__init__(self, outcomes=['start'])

	def execute(self, userdata):
		create_param('path')
		key = raw_input('Type START to go')
		if key == 'START' or key == 'start':
			file = open('/home/nuhanishat/new_kinova_ws/src/kinova-ros'+
'/kinova_demo/nodes/kinova_demo/path.txt', "r")
			lines = file.readlines()
			count = 0
			path = {}

			for line in lines:
				data = line.split()
				points = []
				for j in range(len(data)):
					points.append(float(data[j]))
				path[count] = points
				count += 1

			rospy.set_param('path', path)

			return "start"

