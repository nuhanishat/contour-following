#! usr/bin/env python

import math
import copy
import numpy as np 
import matplotlib.pyplot as plt 


def generate_path():
	y = -0.4

	x = np.linspace(-2, 2, 40)

	z = [float(0.1*math.sin(i) + 0.4) for i in x]

	z_formatted = [round(i, 3) for i in z]
	x_formatted = [float("{:.3f}".format(0.1*i)) for i in x]

	path = [[point_x, y, point_z] for point_x, point_z in zip(x_formatted, z_formatted)]

	# flipped_z = z_formatted[::-1]
	flipped_x = x_formatted[::-1]
	# plt.plot(flipped_x, z_formatted)
	# plt.show()

	file = open('/home/nuhanishat/new_kinova_ws/src/kinova-ros/kinova_demo/nodes/kinova_demo/path.txt', "w")
	

	for i in range(len(path)):
		file.write(str(path[i][0])+' '+str(path[i][1])+' '+ str(path[i][2])+
' '+'0 180 90 \n')

	file.close()

	return path

if __name__ == '__main__':
	path = generate_path()