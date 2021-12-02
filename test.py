#! usr/bin/env python

import rospy
import sys
import actionlib
from kinova_msgs.msg import ArmPoseAction, ArmPoseGoal
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import String, Float32, Header
import math
from kinova_msgs.srv import *
import tf.transformations
import copy
import numpy as np
import matplotlib.pyplot as plt


def quaternion_pose_client(pose):
	"""Takes a list of pose with orientation in degrees
	and sends it to the pose action server """
	action_server = '/j2s7s300_driver/pose_action/tool_pose'
	client = actionlib.SimpleActionClient(action_server, ArmPoseAction)
	client.wait_for_server()

	goal = ArmPoseGoal()
	goal.pose.header = Header(frame_id='j2s7s300_link_base')
	goal.pose.pose.position = Point(x=pose[0], y=pose[1], z=pose[2])

	if len(pose) == 6:
		quat = tf.transformations.quaternion_from_euler(math.radians(pose[3]), math.radians(pose[4]), 
math.radians(pose[5]))
		goal.pose.pose.orientation = Quaternion(x= quat[0], y=quat[1], z=quat[2], w=quat[3])
		rospy.loginfo('Sending goal pose: {0}'.format(goal.pose.pose))

		client.send_goal(goal)

		if client.wait_for_result(rospy.Duration(200.0)):
			return client.get_result()
		else:
			client.cancel_all_goals()
			rospy.loginfo('Timed out. Goal cancelled')
			return None
	else:
		rospy.logwarn('Number of elements must be 6 and orientation must be in degrees')


def generate_path():
	y = 0

	x = np.linspace(0.05, 0.3, 20)

	z = [float(0.05*math.tanh((20*i) - 4) + 0.1) for i in x]

	z_formatted = [round(i, 3) for i in z]
	x_formatted = [float("{:.3f}".format(i)) for i in x]

	flipped_z = z_formatted[::-1]
	flipped_x = x_formatted[::-1]

	path = [[point_x, y, point_z] for point_x, point_z in zip(x_formatted, flipped_z)]
	plt.plot(flipped_x, z_formatted)


	plt.show()

	return path

	


if __name__ == '__main__':
	rospy.init_node('test_client', anonymous=True)


	path = generate_path()
	# print(path)

	start = [-0.05, -0.55, 0.25, 90, 0, 0]


	for i in range(len(path)):
		waypoint = path[i]
		goal = [round(waypoint[0] + start[0], 3), waypoint[1] + start[1], waypoint[2] + start[2], 90, 0, 0]
		rospy.loginfo('Sending goal' + str(i))
		quaternion_pose_client(goal)

	# waypoint = [-0.3, -0.45, 0.25, 0, 180, 0]
	# waypoint = [-0.053, -0.45, 0.25, 90, 0, 0]

	# waypoint = [0.1, -0.2646, 0.5029, 90, 0, 0]
	# rospy.loginfo('Sending goal' + str(waypoint))
	# quaternion_pose_client(waypoint)


""" orientation: 
    x: 0.0233123525977
    y: -0.707737445831
    z: 0.705777108669
    w: 0.0210492499173 """


	
	# try:
	# 	pose = [0.4, -0.4, 0.2, 0, 180, -90]

	# 	count = 0
	# 	while count <= 16:
	# 		goal = copy.deepcopy(pose)
	# 		goal[0] -= 0.025
	# 		count += 1
	# 		rospy.loginfo('Sending goal' + str(count))
	# 		quaternion_pose_client(goal)
	# 		pose = goal

	# except rospy.ROSInterruptException:
	# 	print('Something went wrong')

	# rospy.spin()



""""  Starting Pose:

  x: 0.171853259206
    y: -0.395292758942
    z: 0.177158087492
  orientation: 
    x: 0.755912423134
    y: 0.653237640858
    z: -0.0430952198803
    w: 0.00444791465998
"""