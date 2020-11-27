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

from scipy.spatial.transform import Rotation as R

filename = "sim_test_data/testdata_cont.txt"

time_ref = list()
actual_pos = list()
desired_pos = list()
pos_err = list()
omegac_hat = list()					# matrix
omegac_hat_resize = list()
omegac = list()
rot_err = list() 					# matrix
rot_err_resize = list()
ang_vel_err = list()
omega_hat = list() 					# matrix
omega_hat_resize = list()
omegac_dot = list()
desired_thrust_mag = list()
desired_moments = list()
desired_rotor_rates = list()

# sim_time
# actual_pos
# desired_pos
# position_error
# omegac_hat
# omegac
# rot_err
# ang_vel_err
# omega_hat
# omegac_dot
# des_thrust_mag
# desired_moments
# desired_rotor_rates

def read_data_in():
	with open(filename, 'r') as reader:
		i = 0
		line_cnt = 20
		for line in reader:
			line = line.replace("\n", "")
			if (i%line_cnt) == 0: 					# time
				time_ref.append(float(line))
			elif (i%line_cnt) == 1: 							# actual_pos
				# print(line)
				# print(format_list(line))
				actual_pos.append(format_list(line))
				# print(actual_pos)
				# time.sleep(1.0)
			elif (i%line_cnt) == 2: 							# desired_pos
				desired_pos.append(format_list(line))
			elif (i%line_cnt) == 3: 							# pos_err
				pos_err.append(format_list(line))
			elif (i%line_cnt) == 4: 							# omegac_hat
				omegac_hat.append(line)
			elif (i%line_cnt) == 5:
				omegac_hat.append(line)
			elif (i%line_cnt) == 6:
				omegac_hat.append(line)
			elif (i%line_cnt) == 7: 							# omegac
				omegac.append(format_list(line))
			elif (i%line_cnt) == 8: 							# rot_err
				rot_err.append(line)
			elif (i%line_cnt) == 9:
				rot_err.append(line)
			elif (i%line_cnt) == 10:
				rot_err.append(line)
			elif (i%line_cnt) == 11:							# ang_vel_err
				ang_vel_err.append(format_list(line))
			elif (i%line_cnt) == 12: 							# omega_hat
				omega_hat.append(line)
			elif (i%line_cnt) == 13:
				omega_hat.append(line)
			elif (i%line_cnt) == 14:
				omega_hat.append(line)
			elif (i%line_cnt) == 15: 							# omegac_dot
				omegac_dot.append(format_list(line))
			elif (i%line_cnt) == 16: 							# desired_thrust_mag
				desired_thrust_mag.append(format_list(line))
			elif (i%line_cnt) == 17: 							# desired_moments
				desired_moments.append(format_list(line))
			elif (i%line_cnt) == 18: 							# desired_rotor_rates
				desired_rotor_rates.append(format_list(line))
			elif (i%line_cnt) == 19:
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


def resize_data_in():
	global time_ref
	global actual_pos
	global desired_pos
	global pos_err
	global omegac_hat				# matrix
	global omegac_hat_resize
	global omegac
	global rot_err 					# matrix
	global rot_err_resize
	global ang_vel_err
	global omega_hat 				# matrix
	global omega_hat_resize
	global omegac_dot
	global desired_thrust_mag
	global desired_moments
	global desired_rotor_rates


	omegac_hat_resize = format_matrix(omegac_hat)
	rot_err_resize = format_matrix(rot_err)
	omega_hat_resize = format_matrix(omega_hat)

	min_size = min(len(time_ref), len(actual_pos), len(desired_pos), len(pos_err), len(omegac_hat_resize), len(rot_err_resize), len(ang_vel_err), 
		len(omegac_hat_resize), len(omegac_dot), len(desired_thrust_mag), len(desired_moments), len(desired_rotor_rates))

	time_ref = time_ref[0:min_size]
	time_ref = np.array(time_ref)[0:min_size]

	actual_pos = actual_pos[0:min_size]
	actual_pos = np.array(actual_pos)[0:min_size,:]
	
	desired_pos = desired_pos[0:min_size]
	desired_pos = np.array(desired_pos)[0:min_size,:]
	
	pos_err = pos_err[0:min_size]
	pos_err = np.array(pos_err)[0:min_size,:]
	
	omegac_hat_resize = omegac_hat_resize[0:min_size] 								# matrix
	omegac_hat_resize = np.array(omegac_hat_resize)[0:min_size]
	
	omegac = omegac[0:min_size]
	omegac = np.array(omegac)[0:min_size]
	
	rot_err_resize = rot_err_resize[0:min_size] 									# matrix
	rot_err_resize = np.array(rot_err_resize)[0:min_size]
	# test = R.from_matrix(rot_err_resize[100])
	# print(test)

	ang_vel_err = ang_vel_err[0:min_size]
	ang_vel_err = np.array(ang_vel_err)[0:min_size,:]

	omega_hat_resize = omega_hat_resize[0:min_size] 								# matrix
	omega_hat_resize = np.array(omegac_hat_resize)[0:min_size]

	omegac_dot = omegac_dot[0:min_size]
	omegac_dot = np.array(omegac_dot)[0:min_size]
	
	desired_thrust_mag = desired_thrust_mag[0:min_size]
	desired_thrust_mag = np.array(desired_thrust_mag)[0:min_size,:]
	
	desired_moments = desired_moments[0:min_size]
	desired_moments = np.array(desired_moments)[0:min_size,:]

	desired_rotor_rates = desired_rotor_rates[0:min_size]
	desired_rotor_rates = np.array(desired_rotor_rates)[0:min_size,:]


def visualize_results():
	# plot position data
	plt.figure(200)

	plt.subplot(1,2,1)
	plt.plot(time_ref[:], desired_pos[:,0], label='Des x')
	plt.plot(time_ref[:], desired_pos[:,1], label='Des y')
	plt.plot(time_ref[:], desired_pos[:,2], label='Des z')
	plt.plot(time_ref[:], actual_pos[:,0], label='Act x')
	plt.plot(time_ref[:], actual_pos[:,1], label='Act y')
	plt.plot(time_ref[:], actual_pos[:,2], label='Act z')	

	plt.xlabel('Sim time (s)')
	plt.ylabel('Position (m)')
	plt.title("Position over time")
	plt.legend()
	plt.grid(True)

	# # plot attitude data
	# plt.subplot(1,2,2)
	# plt.plot(time, desired_att[:,0], label='Des roll')
	# plt.plot(time, desired_att[:,1], label='Des pitch')
	# plt.plot(time, desired_att[:,2], label='Des yaw')
	# plt.plot(time, actual_att[:,0], label='Act roll')
	# plt.plot(time, actual_att[:,1], label='Act pitch')
	# plt.plot(time, actual_att[:,2], label='Act yaw')	

	# plt.xlabel('Sim time (s)')
	# plt.ylabel('Degrees (rad)')
	# plt.title("Attitude over time")
	# plt.legend()
	# plt.grid(True)


	plt.figure(300)
	plt.subplot(1,2,1)
	plt.plot(time_ref[:], desired_thrust_mag[:], label="thrust mag")
	plt.plot(time_ref[:], desired_rotor_rates[:,1], label="motor 1")
	plt.plot(time_ref[:], desired_rotor_rates[:,1], label="motor 2")
	plt.plot(time_ref[:], desired_rotor_rates[:,2], label="motor 3")
	plt.plot(time_ref[:], desired_rotor_rates[:,3], label="motor 4")

	plt.xlabel('Sim time (s)')
	plt.ylabel('Motor speed (rad/s)')
	plt.title("Motor speed over time")
	plt.legend()
	plt.grid(True)

	# plt.subplot(1,2,2)
	# plt.plot(time, attitude_deltas[:,0], label="roll delta")
	# plt.plot(time, attitude_deltas[:,1], label="pitch delta")
	# plt.plot(time, attitude_deltas[:,2], label="yaw delta")

	# plt.xlabel('Sim time (s)')
	# plt.ylabel('Attitude Delta (rad)')
	# plt.title("Attitude deltas over time")
	# plt.legend()
	# plt.grid(True)

	plt.tight_layout()
	plt.show()


read_data_in()
resize_data_in()
visualize_results()
