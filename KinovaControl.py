#! usr/bin/env python

import rospy
import sys
import actioplib
from kinova_msgs.msg import ArmPoseAction, ArmPoseGoal, PoseVelocity
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import String, Float32, Header
import math
from kinova_msgs.srv import *
import copy
import numpy as np 
import matplotlib.pyplot as plt 


class KinovaControl():
	def __init__(self):
		"""Uses the kinova control modules to send position and velocity
		commands to the robot using kinova_msgs"""

		# The position action server definition and client
		self.pose_action_server = '/j2s7s300_driver/pose_action/tool_pose'
		self.pose_action_client = actionlib.SimpleActionClient(self.pose_action_server, ArmPoseAction)

		# Wait for client to be active
		self.pose_action_client.wait_for_server()

		# Create a arm pose goal for postion action client
		self.pose_goal = ArmPoseGoal()
		self.pose_goal.header = Header(frame_id='j2s7s300_link_base')

		# Velocity command topic definition
		self.velocity_topic = '/j2s7s300_driver/in/cartesian_velocity'

		# Velocity message
		self.velocity = PoseVelocity()

		# Create the velocity publisher
		self.velocity_publisher  = rospy.Publisher(self.velocity_topic, PoseVelocity, queue_size=1)

		# Define a velocity publishing rate
		self.rate = rospy.Rate(100) #Hz



	def pose_action_client(self, pose):
		'''This is the kinova pose client. 
		It takes in the pose in a list e.g [x,y,z,roll,pitch,yaw] where 
		the angle are in degrees'''

		self.pose_goal.pose.pose.position = Point(x=pose[0], y=pose[1], z=pose[2])

		if len(pose) == 6:
			quat = tf.transformations.qauternion_from_euler(math.radians(pose[3]), 
math.radians(pose[4]), math.radians(pose[5]) )
			self.pose_goal.pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
			rospy.loginfo('Sending goal pose: {0}'.format(self.pose_goal.pose))

			self.pose_action_client.send_goal(self.pose_goal)

			if self.pose_action_client.wait_for_result(rospy.Duration(200.0)):
				return self.pose_action_client.get_result()

			else:
				self.pose_action_client.cancel_all_goals()
				rospy.loginfo('Timed out. Goal cancelled')
				return None

		else:
			rospy.logwarn('Number of elements must be 6 and orientation must be in degrees')



	def cartesian_velocity_command(self, velocity, duration):
		'''This is the cartesian velocity publisher. 
		It takes in the twist vector in a list e.g [linear_x, linear_y, linear_z, angular_x, angular_y, 
		angular_z], duration is in seconds'''

		self.velocity.twist_linear_x = velocity[0]
		self.velocity.twist_linear_y = velocity[1]
		self.velocity.twist_linear_z = velocity[2]
		self.velocity.twist_angular_x = velocity[3]
		self.velocity.twist_angular_y = velocity[4]
		self.velocity.twist_angular_z = velocity[5]

		count = 0

		# Publish for the duration asked
		while count < 100*duration:
			count += 1
			self.velocity_publisher.publish(self.velocity)
			self.rate.sleep()


	def distance_based_velocity_command(self, error_vector):
		"""This is also a cartesian velocity publisher, but uses the error vector, which
		is simply the delta x, y, z to make 


if __name__ == '__main__':
	rospy.init_node('Kinova_command_test', anonymous=True)

	'''Write any test code here to check if you are able to connect to the arm
	and send pose or velocity commands'''




