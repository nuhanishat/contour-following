#! /usr/bin/env python

# Author: Nuha Nishat
# Date: 11/12/20

import rospy
import sys


def create_param(param):
	rospy.set_param(param, {})


def save_new_place(param, place_name, pose):
	places = rospy.get_param(param)
	places.update({place_name : pose})
	rospy.set_param(param, places)



if __name__ == '__main__':
	create_param()
	save_new_place('wonderland', [1,2,3])

	# Check
	places = rospy.get_param('places')
	print(places)
