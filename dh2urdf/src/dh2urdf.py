#! /usr/bin/python

import json
import math

from tf.transformations import *

xaxis, yaxis, zaxis = (1, 0, 0), (0, 1, 0), (0, 0, 1)

if __name__ == '__main__':
	params = {}
	results = ''
	with open('/home/kaczmi/catkin_ws/src/anro-kaczmarek_kobylinski/dh2urdf/dh.json', 'r') as file:
		params = json.loads(file.read())
	with open('../urdf.yaml', 'w') as file:
		for key in params.keys():
			a, d, al, th = params[key]
			a, d, al, th = float(a), float(d), float(al), float(th)

			tz = translation_matrix((0, 0, d))
			rz = rotation_matrix(th, zaxis)
			tx = translation_matrix((a, 0, 0))
			rx = rotation_matrix(al, xaxis)

			matrix = concatenate_matrices(tz, rz, tx, rx)

			rpy = euler_from_matrix(matrix)
			xyz = translation_from_matrix(matrix)

			l_len=math.sqrt(a*a+d*d)

			file.write(key + ":\n")
			file.write("  j_xyz: {} {} {}\n".format(*xyz))
			file.write("  j_rpy: {} {} {}\n".format(*rpy))
			file.write("  l_xyz: {} {} {}\n".format(xyz[0]/2, xyz[1]/2, xyz[2]/2))
			file.write("  l_rpy: {} {} {}\n".format(*rpy))
			file.write("  l_len: {}\n".format(l_len))
			file.write("  l_rot: {}\n".format(th))
			
