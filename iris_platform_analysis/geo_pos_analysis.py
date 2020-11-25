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

def read_data_in():
	with open(filename, 'r') as reader:
		i = 0
		line_cnt = 20
		for line in reader:
			# print(i%line_cnt)
			# print(line)
			# time.sleep(0.5)
			line = line.replace("\n", "")
			if (i%line_cnt) == 0: 					# time
				time_ref.append(line)
			elif (i%line_cnt) == 1: 							# actual_pos
				actual_pos.append(line)
			elif (i%line_cnt) == 2: 							# desired_pos
				desired_pos.append(line)
			elif (i%line_cnt) == 3: 							# pos_err
				pos_err.append(line)
			elif (i%line_cnt) == 4: 							# omegac_hat
				omegac_hat.append(line)
			elif (i%line_cnt) == 5:
				omegac_hat.append(line)
			elif (i%line_cnt) == 6:
				omegac_hat.append(line)
			elif (i%line_cnt) == 7: 							# omegac
				omegac.append(line)
			elif (i%line_cnt) == 8: 							# rot_err
				rot_err.append(line)
			elif (i%line_cnt) == 9:
				rot_err.append(line)
			elif (i%line_cnt) == 10:
				rot_err.append(line)
			elif (i%line_cnt) == 11:							# ang_vel_err
				ang_vel_err.append(line)
			elif (i%line_cnt) == 12: 							# omega_hat
				omega_hat.append(line)
			elif (i%line_cnt) == 13:
				omega_hat.append(line)
			elif (i%line_cnt) == 14:
				omega_hat.append(line)
			elif (i%line_cnt) == 15: 							# omegac_dot
				omegac_dot.append(line)
			elif (i%line_cnt) == 16: 							# desired_thrust_mag
				desired_thrust_mag.append(line)
			elif (i%line_cnt) == 17: 							# desired_moments
				desired_moments.append(line)
			elif (i%line_cnt) == 18: 							# desired_rotor_rates
				desired_rotor_rates.append(line)
			elif (i%line_cnt) == 19:
				pass
			else:
				print("ERROR")
				return
		
# 			line = reader.readline()
# 			line = line.replace("\n", "")
			i = i + 1

# 			

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

	j = 0
	temp = list()
	for i in range(0, len(omegac_hat)):
		if j < 3:
			temp.append(omegac_hat[i])
			j = j + 1
		else:
			omegac_hat_resize.append(temp)
			temp = list()
			temp.append(omegac_hat[i])
			j = 1

	j = 0
	temp = list()
	for i in range(0, len(rot_err)):
		if j < 3:
			temp.append(rot_err[i])
			j = j + 1
		else:
			rot_err_resize.append(temp)
			temp = list()
			temp.append(rot_err[i])
			j = 1

	j = 0
	temp = list()
	for i in range(0, len(omega_hat)):
		if j < 3:
			temp.append(omega_hat[i])
			j = j + 1
		else:
			omega_hat_resize.append(temp)
			temp = list()
			temp.append(omega_hat[i])
			j = 1

	print("omegac_hat", len(omegac_hat_resize))
	print("rot_err", len(rot_err_resize))
	print("omega hat", len(omega_hat_resize))

	min_size = min(len(time_ref), len(actual_pos), len(desired_pos), len(pos_err), len(omegac_hat_resize), len(rot_err_resize), len(ang_vel_err), 
		len(omegac_hat_resize), len(omegac_dot), len(desired_thrust_mag), len(desired_moments), len(desired_rotor_rates))
	print(min_size)

	time_ref = np.array(time_ref)
	global actual_pos
	global desired_pos
	global pos_err
	global omegac_hat_resize
	global omegac
	global rot_err_resize
	global ang_vel_err
	global omega_hat_resize
	global omegac_dot
	global desired_thrust_mag
	global desired_moments
	global desired_rotor_rates

	# time = np.array(time)
	# desired_pos = np.array(desired_pos)
	# actual_pos = np.array(actual_pos)
	# desired_att = np.array(desired_att)
	# actual_att = np.array(actual_att)
	# desired_thrust = np.array(desired_thrust)
	# attitude_deltas = np.array(attitude_deltas)


# def visualize_results():
# 	# plot position data
# 	plt.figure(200)

# 	plt.subplot(1,2,1)
# 	plt.plot(time, desired_pos[:,0], label='Des x')
# 	plt.plot(time, desired_pos[:,1], label='Des y')
# 	plt.plot(time, desired_pos[:,2], label='Des z')
# 	plt.plot(time, actual_pos[:,0], label='Act x')
# 	plt.plot(time, actual_pos[:,1], label='Act y')
# 	plt.plot(time, actual_pos[:,2], label='Act z')	

# 	plt.xlabel('Sim time (s)')
# 	plt.ylabel('Position (m)')
# 	plt.title("Position over time")
# 	plt.legend()
# 	plt.grid(True)

# 	# plot attitude data
# 	plt.subplot(1,2,2)
# 	plt.plot(time, desired_att[:,0], label='Des roll')
# 	plt.plot(time, desired_att[:,1], label='Des pitch')
# 	plt.plot(time, desired_att[:,2], label='Des yaw')
# 	plt.plot(time, actual_att[:,0], label='Act roll')
# 	plt.plot(time, actual_att[:,1], label='Act pitch')
# 	plt.plot(time, actual_att[:,2], label='Act yaw')	

# 	plt.xlabel('Sim time (s)')
# 	plt.ylabel('Degrees (rad)')
# 	plt.title("Attitude over time")
# 	plt.legend()
# 	plt.grid(True)


# 	plt.figure(300)
# 	plt.subplot(1,2,1)
# 	plt.plot(time, desired_thrust[:,0], label="motor 1")
# 	plt.plot(time, desired_thrust[:,1], label="motor 2")
# 	plt.plot(time, desired_thrust[:,2], label="motor 3")
# 	plt.plot(time, desired_thrust[:,3], label="motor 4")

# 	plt.xlabel('Sim time (s)')
# 	plt.ylabel('Motor speed (rad/s)')
# 	plt.title("Motor speed over time")
# 	plt.legend()
# 	plt.grid(True)

# 	plt.subplot(1,2,2)
# 	plt.plot(time, attitude_deltas[:,0], label="roll delta")
# 	plt.plot(time, attitude_deltas[:,1], label="pitch delta")
# 	plt.plot(time, attitude_deltas[:,2], label="yaw delta")

# 	plt.xlabel('Sim time (s)')
# 	plt.ylabel('Attitude Delta (rad)')
# 	plt.title("Attitude deltas over time")
# 	plt.legend()
# 	plt.grid(True)

# 	plt.tight_layout()
# 	plt.show()


read_data_in()
resize_data_in()

# print("time_ref", len(time_ref))
# print("acutal_pos", len(actual_pos)) 
# print("desired_pos", len(desired_pos))
# print("pos_err", len(pos_err))
# print("omegac_hat", len(omegac_hat))
# print("omegac", len(omegac))
# print("rot_err", len(rot_err))
# print("ang_vel_err", len(ang_vel_err))
# print("omega_hat", len(omega_hat))
# print("omegac_dot", len(omegac_dot))
# print("desired_thrust_mag", len(desired_thrust_mag))
# print("desired_moments", len(desired_moments))
# print("desired_rotor_rates", len(desired_rotor_rates))


# # https://stackoverflow.com/questions/37600711/pandas-split-column-into-multiple-columns-by-comma
# # https://stackoverflow.com/questions/1614236/in-python-how-do-i-convert-all-of-the-items-in-a-list-to-floats?rq=1
# df = pd.DataFrame(time, columns=["Time"])
# df['Desired Position'] = desired_pos
# df["Desired Position"] = df["Desired Position"].str.split(" ")
# df['Actual Position'] = actual_pos
# df["Actual Position"] = df["Actual Position"].str.split(" ")
# df['Desired Attitude'] = desired_att
# df["Desired Attitude"] = df["Desired Attitude"].str.split(" ")
# df['Actual Attitude'] = actual_att
# df["Actual Attitude"] = df["Actual Attitude"].str.split(" ")
# df["Desired Thrust"] = desired_thrust
# df["Desired Thrust"] = df["Desired Thrust"].str.split(" ")
# df["Attitude Deltas"] = attitude_deltas
# df["Attitude Deltas"] = df["Attitude Deltas"].str.split(" ")

# df = df.drop([0])
# df = df.drop([len(df)])

# for i in range(len(df)):
# 	for j in range(len(df.columns)):
# 		if j == 0:
# 			df.iloc[i,j] = float(df.iloc[i,j])
# 			continue
# 		temp = []
# 		while df.iloc[i,j]:
# 			x = df.iloc[i,j].pop()
# 			if not x:
# 				pass
# 			else:
# 				temp.insert(0, float(x)) 			# pop pulls elements off the end of an array, this corrects the order

# 		df.iloc[i,j] = temp


# # SOMETIMES THIS HAPPENS
# # so the values in desired_pos are strings... not sure why
# # also, maybe one of the rows is not 1x3 which is goofing things
# df["Time"] = df["Time"].astype("float")
# df.to_csv("./sim_test_data/file_cleaned.csv", index=False)

# time = np.array(df["Time"].tolist())
# desired_pos = np.array(df["Desired Position"].values.tolist())
# actual_pos = np.array(df["Actual Position"].values.tolist())
# desired_att = np.array(df["Desired Attitude"].values.tolist())
# actual_att = np.array(df["Actual Attitude"].values.tolist())
# desired_thrust = np.array(df["Desired Thrust"].values.tolist())
# attitude_deltas = np.array(df["Attitude Deltas"].values.tolist())

# # print(type(time[0]))
# # print(type(desired_pos[0]))
# # print(type(actual_pos[0]))
# # print(type(desired_att[0]))
# # print(type(actual_att[0]))

# visualize_results()