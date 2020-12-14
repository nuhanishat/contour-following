#! usr/bin/env python

import math
import copy
import numpy as np 
import matplotlib.pyplot as plt 


def generate_path():
	z = 0.2

	x = np.linspace(-2, 2, 40)

	y = [float(0.1*math.sin(i) + 0.3) for i in x]

	y_formatted = [round(i, 3) for i in y]
	x_formatted = [float("{:.3f}".format(0.1*i)) for i in x]

	

	# flipped_z = z_formatted[::-1]
	flipped_x = x_formatted[::-1]

	path = [[point_x, point_y, z] for point_x, point_y in zip(flipped_x, y_formatted)]

	plt.plot(flipped_x, y_formatted)
	plt.show()

	file = open('/home/nuhanishat/new_kinova_ws/src/kinova-ros/kinova_demo/nodes/kinova_demo/path.txt', "w")
	

	for i in range(len(path)):
		file.write(str(path[i][0])+' '+str(path[i][1])+' '+ str(path[i][2])+
' '+'0 180 90 \n')

	file.close()

	path_for_vel = [[point_x, point_y, z] for point_x, point_y in zip(x[::-1], y)]

	return path_for_vel


def generate_velocity(path):
	"""Generate the velocity of the path to be used for twist command"""

	# Create a numpy array of path

	x_np = np.array([path[i][0] for i in range(len(path))])
	y_np = np.array([path[i][1] for i in range(len(path))])

	velocity_x = np.gradient(x_np)
	velocity_y = np.gradient(y_np)

	print('velocity_x: ', velocity_x)
	print('velocity_y: ', velocity_y)

	velocity = np.zeros((2, len(velocity_x)))

	for i in range(len(velocity_x)):
		velocity[0][i] = velocity_x[i]
		velocity[1][i] = velocity_y[i]

	# print('velocity: ', velocity)

	# file = open('/home/nuhanishat/new_kinova_ws/src/kinova-ros/kinova_demo/nodes/kinova_demo/velocity.npy', "w")

	# with file:
	# 	np.save(file, velocity)

	# file.close()

	# plt.plot(velocity_x)
	# plt.plot(velocity_y)
	# plt.show()
	return velocity


if __name__ == '__main__':
	path = generate_path()
	velocity = generate_velocity(path)