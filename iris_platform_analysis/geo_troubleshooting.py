# Plots the positions and attitudes of a quadcopter vs. desired. Used to help tune the controllers
# 
# controller tuning idea is 1) takeoff to hover 2) stabilize in hover 3) change yaw a little bit 4) roll/pitch 5) x, y -- THIS NEEDS TO CHANGE FOR GEOMETRIC CONTROLLERS
# 
# By: Patrick Ledzian
# Date: 23 November 2020
# 

import numpy as np
import matplotlib.pyplot as plt 
import pandas as pd
import time
from pytransform3d.rotations import plot_basis
from pytransform3d.trajectories import plot_trajectory
import math

# from scipy.spatial.transform import Rotation as R

filename = "sim_test_data/troubleshooting.txt"

time_ref = list()
actual_pos = list()
actual_att = list()
desired_rotor_rates = list()
desired_moments = list()
test_desired_moments = list()
test_Rc = list()
test_Rc_resize = list()
test_omegacdot = list()
test_derived_rot = list()
test_derived_rot_resize = list()

# sim_time
# desired_rotor_rates
# test_desired_moments
# test_Rc 					3x3 matrix
# test_omegacdot
# test_derived_rot 			3x3 matrix

def read_data_in():
	with open(filename, 'r') as reader:
		i = 0
		line_cnt = 14
		for line in reader:
			line = line.replace("\n", "")
			if (i%line_cnt) == 0: 					# time
				time_ref.append(float(line))
			elif (i%line_cnt) == 1: 							
				actual_pos.append(format_list(line))
			elif (i%line_cnt) == 2:
				actual_att.append(format_list(line))
			elif (i%line_cnt) == 3: 				
				desired_rotor_rates.append(format_list(line))
			elif(i%line_cnt) == 4:
				desired_moments.append(format_list(line))
			elif(i%line_cnt) == 5:
				test_desired_moments.append(format_list(line))
			elif(i%line_cnt) == 6:
				test_Rc.append(line)
			elif(i%line_cnt) == 7:
				test_Rc.append(line)
			elif(i%line_cnt) == 8:
				test_Rc.append(line)
			elif(i%line_cnt) == 9:
				test_omegacdot.append(format_list(line))
			elif(i%line_cnt) == 10:
				test_derived_rot.append(line)
			elif(i%line_cnt) == 11:
				test_derived_rot.append(line)
			elif(i%line_cnt) == 12:
				test_derived_rot.append(line)
			elif(i%line_cnt) == 13:
				pass
			else:
				print("ERROR")
				return
		
			i = i + 1 			# iterate the parser


def format_list(lst):
	splt = lst.split() 				# splits on the spaces
	mapped_obj = map(float, splt)	# creates map object str->float
	
	return(list(mapped_obj))		# return list of floats


def format_matrix(lst):
	j = 0
	temp = list()
	new_mat = list()
	for i in range(0, len(lst)):
		if j < 3:
			temp.append(format_list(lst[i])) 		# appends list of floats to temp
			j = j + 1
		else:
			new_mat.append(temp) 					# appends nested list to list, eg: matrix
			temp = list()
			temp.append(format_list(lst[i]))
			j = 1

	return(new_mat) 								# return matrix of floats


# TODO:
# 1) figure out Rc axis mismatch
# 2) figure out which data streams are "spiking" and why
# 	- if this is computation time related, maybe update the time slice for each variable (dedicated time slices)
def resize_data_in():
	global time_ref
	global actual_pos
	global actual_att
	global desired_rotor_rates
	global desired_moments
	global test_desired_moments
	global test_Rc 					# matrix
	global test_Rc_resize 			# matrix
	global test_omegacdot 
	global test_derived_rot 		# matrix
	global test_derived_rot_resize 	# matrix

	test_Rc_resize = format_matrix(test_Rc)
	test_derived_rot_resize = format_matrix(test_derived_rot)

	min_size = min(len(time_ref), len(desired_rotor_rates), len(test_desired_moments), len(test_Rc), len(test_omegacdot), len(test_derived_rot), len(desired_moments), len(actual_pos), len(actual_att))

	if min_size == 0:
		print("Invalid 'min_size' -- value is 0")

	time_ref = time_ref[1:min_size]
	time_ref = np.array(time_ref)[1:min_size]

	actual_pos = actual_pos[1:min_size]
	actual_pos = np.array(actual_pos)[1:min_size, :]

	actual_att = actual_att[1:min_size]
	actual_att = np.array(actual_att)[1:min_size, :]

	desired_rotor_rates = desired_rotor_rates[1:min_size]
	desired_rotor_rates = np.array(desired_rotor_rates)[1:min_size,:]

	desired_moments = desired_moments[1:min_size]
	desired_moments = np.array(desired_moments)[1:min_size,:]
	
	test_desired_moments = test_desired_moments[1:min_size]
	test_desired_moments = np.array(test_desired_moments)[1:min_size,:]
	
	test_Rc_resize = test_Rc_resize[1:min_size] 								# matrix
	test_Rc_resize = np.array(test_Rc_resize)[1:min_size]
	
	test_omegacdot = test_omegacdot[1:min_size]
	test_omegacdot = np.array(test_omegacdot)[1:min_size]

	test_derived_rot_resize = test_derived_rot_resize[1:min_size] 								# matrix
	test_derived_rot_resize = np.array(test_derived_rot_resize)[1:min_size]

	# print(test_Rc_resize[:,:])
	# print(test_Rc_resize[:,:][0])
	# print(test_derived_rot_resize[:,:][0])
	
	# for i in range(math.floor(len(test_derived_rot_resize)/100)):
	# 	print(time_ref[i*100])
	# 	print(test_derived_rot_resize[:,:][i*100])
	# 	print("\n")
	# 	time.sleep(0.5)


def visualize_results():
	plt.figure(200)

	# plot time
	plt.subplot(1,2,1)
	plt.plot(time_ref[:])

	plt.xlabel('Data points')
	plt.ylabel('Time (s)')
	plt.title("Time series")
	# plt.legend()
	plt.grid(True)

	# plot motor rates
	plt.subplot(1,2,2)
	plt.plot(time_ref[:], desired_rotor_rates[:,0], label='Motor 1')
	# plt.plot(time_ref[:], desired_rotor_rates[:,1], label='Motor 2')
	# plt.plot(time_ref[:], desired_rotor_rates[:,2], label='Motor 3')
	# plt.plot(time_ref[:], desired_rotor_rates[:,3], label='Motor 4')	

	plt.xlabel('Sim time (s)')
	plt.ylabel('Motor rate (rad/s)')
	plt.title("Motor rates over time")
	plt.legend()
	plt.grid(True)


	plt.figure(300)

	# plot desired moments
	plt.subplot(1,2,1)
	plt.plot(time_ref[:], desired_moments[:,0], label="roll")
	# plt.plot(time_ref[:], desired_moments[:,1], label="pitch")
	# plt.plot(time_ref[:], desired_moments[:,2], label="yaw")

	plt.xlabel('Sim time (s)')
	plt.ylabel('Moments (N)')
	plt.title("Desired moments over time")
	plt.legend()
	plt.grid(True)

	###
	#
	# PLOT THE COORDINATE AXIS ROTATING OVER TIME  --- continue looking for spikes in data / numerical issues
	# 
	###
	# https://rock-learning.github.io/pytransform3d/rotations.html
	# 
	#  TODO: can also plot the error in rotation matrix properties, eg: R orthonormal, R^T = R^-1, det(R) = 1
	#  
	#  deriving euler angles from rotation matrix
	#  https://www.gregslabaugh.net/publications/euler.pdf
	#  
	#  TODO: my time derivatives are probably wrong, this is a fundamental math issue. I need to work these derivatives on paper
	#  

	plt.figure(400)
	ax = plot_basis(R=np.eye(3), ax_s=2)

	scaling = 75
	for i in range(math.floor(len(test_derived_rot_resize)/scaling)):	
	# 	plot_basis(ax=ax, R=test_derived_rot_resize[:,:][i*100], p=actual_pos[i*100], ax_s=0.5)
		plot_basis(ax=ax, R=test_Rc_resize[:,:][i*scaling], p=actual_pos[i*scaling], strict_check=False, ax_s=0.5) 		# FAILS INVERSION TEST
	# 		

	# Plot "trajectory", doesn't look great	
	# combined_traj = np.concatenate((actual_pos, actual_att), axis=1)
	# plot_trajectory(ax=ax, P=combined_traj)	


	# # plot Rc vectors (columns)
	# plt.subplot(1,2,2)
	# # plt.plot(time_ref[:], desired_thrust_mag[:], label="thrust mag")
	# plt.plot(time_ref[:], test_Rc_resize[:,:][0], label="00")
	# # plt.plot(time_ref[:], test_Rc_resize[:,0][1], label="01")
	# # plt.plot(time_ref[:], test_Rc_resize[:,0][2], label="02")

	# plt.xlabel('Sim time (s)')
	# plt.ylabel('Rc value')
	# plt.title("Rc vector over time")
	# plt.legend()
	# plt.grid(True)

	plt.tight_layout()
	plt.show()


read_data_in()
resize_data_in()
visualize_results()
