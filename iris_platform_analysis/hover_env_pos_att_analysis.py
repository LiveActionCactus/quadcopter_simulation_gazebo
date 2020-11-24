# Plots the positions and attitudes of a quadcopter vs. desired. Used to help tune the controllers
# controller tuning idea is 1) takeoff to hover 2) stabilize in hover 3) change yaw a little bit 4) roll/pitch 5) x, y
# 
# By: Patrick Ledzian
# Date: 25 September 2020
# 

import numpy as np
import matplotlib.pyplot as plt 
import pandas as pd

filename = "sim_test_data/test_data.txt"

time = list()
desired_pos = list()
actual_pos = list()
desired_att = list()
actual_att = list()
desired_thrust = list()
attitude_deltas = list()

def read_data_in():
	i = 0
	t = 0
	with open(filename, 'r') as reader:
		line = reader.readline()
		line = line.replace("\n", "")
		while line != '':
			i = i + 1
			t = t + 1
			if i == 1:
				time.append(line)
			elif i == 2:
				# line = line.split(" ")
				# line = list(map(float, line))
				desired_pos.append(line)
			elif i == 3:
				# line = line.split(" ")
				# print(t)
				# print(line)
				# print(float(line[0]))
				# print(list(map(float, line)))
				actual_pos.append(line)
			elif i == 4:
				desired_att.append(line)
			elif i == 5:
				actual_att.append(line)
			elif i == 6:
				desired_thrust.append(line)
			elif i == 7:
				attitude_deltas.append(line)
				i = 0

			else:
				print("ERROR")
				return
		
			line = reader.readline()
			line = line.replace("\n", "")


def resize_data_in():
	global time
	global desired_pos
	global actual_pos
	global desired_att
	global actual_att
	global desired_thrust
	global attitude_deltas

	m = len(time) - len(attitude_deltas)
	j = len(desired_pos) - len(attitude_deltas)
	k = len(actual_pos) - len(attitude_deltas)
	l = len(desired_att) - len(attitude_deltas)
	n = len(actual_att) - len(attitude_deltas)
	p = len(desired_thrust) - len(attitude_deltas)

	if m != 0:
		time = time[:-m]
	if j != 0:
		desired_pos = desired_pos[:-j]
	if k != 0:
		actual_pos = actual_pos[:-k]
	if l != 0:
		desired_att = desired_att[:-l]
	if n != 0:
		actual_att = actual_att[:-n]
	if p != 0:
		desired_thrust = desired_thrust[:-p]

	time = np.array(time)
	desired_pos = np.array(desired_pos)
	actual_pos = np.array(actual_pos)
	desired_att = np.array(desired_att)
	actual_att = np.array(actual_att)
	desired_thrust = np.array(desired_thrust)
	attitude_deltas = np.array(attitude_deltas)


def visualize_results():
	# plot position data
	plt.figure(200)

	plt.subplot(1,2,1)
	plt.plot(time, desired_pos[:,0], label='Des x')
	plt.plot(time, desired_pos[:,1], label='Des y')
	plt.plot(time, desired_pos[:,2], label='Des z')
	plt.plot(time, actual_pos[:,0], label='Act x')
	plt.plot(time, actual_pos[:,1], label='Act y')
	plt.plot(time, actual_pos[:,2], label='Act z')	

	plt.xlabel('Sim time (s)')
	plt.ylabel('Position (m)')
	plt.title("Position over time")
	plt.legend()
	plt.grid(True)

	# plot attitude data
	plt.subplot(1,2,2)
	plt.plot(time, desired_att[:,0], label='Des roll')
	plt.plot(time, desired_att[:,1], label='Des pitch')
	plt.plot(time, desired_att[:,2], label='Des yaw')
	plt.plot(time, actual_att[:,0], label='Act roll')
	plt.plot(time, actual_att[:,1], label='Act pitch')
	plt.plot(time, actual_att[:,2], label='Act yaw')	

	plt.xlabel('Sim time (s)')
	plt.ylabel('Degrees (rad)')
	plt.title("Attitude over time")
	plt.legend()
	plt.grid(True)


	plt.figure(300)
	plt.subplot(1,2,1)
	plt.plot(time, desired_thrust[:,0], label="motor 1")
	plt.plot(time, desired_thrust[:,1], label="motor 2")
	plt.plot(time, desired_thrust[:,2], label="motor 3")
	plt.plot(time, desired_thrust[:,3], label="motor 4")

	plt.xlabel('Sim time (s)')
	plt.ylabel('Motor speed (rad/s)')
	plt.title("Motor speed over time")
	plt.legend()
	plt.grid(True)

	plt.subplot(1,2,2)
	plt.plot(time, attitude_deltas[:,0], label="roll delta")
	plt.plot(time, attitude_deltas[:,1], label="pitch delta")
	plt.plot(time, attitude_deltas[:,2], label="yaw delta")

	plt.xlabel('Sim time (s)')
	plt.ylabel('Attitude Delta (rad)')
	plt.title("Attitude deltas over time")
	plt.legend()
	plt.grid(True)

	# plt.figure(400)
	# # plt.plot(time, (((desired_att[:,1]-actual_att[:,1]).square()).divide(len(desired_att[:,1]))).sqrt())
	# err = desired_att[:,1]-actual_att[:,1]
	# test = np.sqrt(err)
	# test1 = np.divide(test, len(desired_att[:,1]))
	# test2 = np.sqrt(test1)
	# plt.plot(time, np.square(err))


	# plt.xlabel('Sim time (s)')
	# plt.ylabel('RMS')
	# plt.title("RMS over time")
	# plt.legend()
	# plt.grid(True)

	plt.tight_layout()
	plt.show()


read_data_in()
resize_data_in()

# https://stackoverflow.com/questions/37600711/pandas-split-column-into-multiple-columns-by-comma
# https://stackoverflow.com/questions/1614236/in-python-how-do-i-convert-all-of-the-items-in-a-list-to-floats?rq=1
df = pd.DataFrame(time, columns=["Time"])
df['Desired Position'] = desired_pos
df["Desired Position"] = df["Desired Position"].str.split(" ")
df['Actual Position'] = actual_pos
df["Actual Position"] = df["Actual Position"].str.split(" ")
df['Desired Attitude'] = desired_att
df["Desired Attitude"] = df["Desired Attitude"].str.split(" ")
df['Actual Attitude'] = actual_att
df["Actual Attitude"] = df["Actual Attitude"].str.split(" ")
df["Desired Thrust"] = desired_thrust
df["Desired Thrust"] = df["Desired Thrust"].str.split(" ")
df["Attitude Deltas"] = attitude_deltas
df["Attitude Deltas"] = df["Attitude Deltas"].str.split(" ")

df = df.drop([0])
df = df.drop([len(df)])

for i in range(len(df)):
	for j in range(len(df.columns)):
		if j == 0:
			df.iloc[i,j] = float(df.iloc[i,j])
			continue
		temp = []
		while df.iloc[i,j]:
			x = df.iloc[i,j].pop()
			if not x:
				pass
			else:
				temp.insert(0, float(x)) 			# pop pulls elements off the end of an array, this corrects the order

		df.iloc[i,j] = temp


# SOMETIMES THIS HAPPENS
# so the values in desired_pos are strings... not sure why
# also, maybe one of the rows is not 1x3 which is goofing things
df["Time"] = df["Time"].astype("float")
df.to_csv("./sim_test_data/file_cleaned.csv", index=False)

time = np.array(df["Time"].tolist())
desired_pos = np.array(df["Desired Position"].values.tolist())
actual_pos = np.array(df["Actual Position"].values.tolist())
desired_att = np.array(df["Desired Attitude"].values.tolist())
actual_att = np.array(df["Actual Attitude"].values.tolist())
desired_thrust = np.array(df["Desired Thrust"].values.tolist())
attitude_deltas = np.array(df["Attitude Deltas"].values.tolist())

# print(type(time[0]))
# print(type(desired_pos[0]))
# print(type(actual_pos[0]))
# print(type(desired_att[0]))
# print(type(actual_att[0]))

visualize_results()