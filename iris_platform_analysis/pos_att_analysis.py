# Plots the positions and attitudes of a quadcopter vs. desired. Used to help tune the controllers
# controller tuning idea is 1) takeoff to hover 2) stabilize in hover 3) change yaw a little bit 4) roll/pitch 5) x, y
# 
# By: Patrick Ledzian
# Date: 25 September 2020
# 

import numpy as np
import matplotlib.pyplot as plt 
import pandas as pd

filename = "sim_test_data/test_data_hover.txt"

time = list()
desired_pos = list()
actual_pos = list()
desired_att = list()
actual_att = list()

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

	m = len(time) - len(actual_att)
	j = len(desired_pos) - len(actual_att)
	k = len(actual_pos) - len(actual_att)
	l = len(desired_att) - len(actual_att)

	if m != 0:
		time = time[:-m]
	if j != 0:
		desired_pos = desired_pos[:-j]
	if k != 0:
		actual_pos = actual_pos[:-k]
	if l != 0:
		desired_att = desired_att[:-l]

	time = np.array(time)
	desired_pos = np.array(desired_pos)
	actual_pos = np.array(actual_pos)
	desired_att = np.array(desired_att)
	actual_att = np.array(actual_att)


def visualize_results():
	# plot position data
	plt.subplot(1,2,1)
	plt.plot(time, desired_pos[:,0])
	plt.plot(time, desired_pos[:,1])
	plt.plot(time, desired_pos[:,2])
	plt.plot(time, actual_pos[:,0])
	plt.plot(time, actual_pos[:,1])
	plt.plot(time, actual_pos[:,2])	

	plt.xlabel('Sim time (s)')
	plt.ylabel('Position (m)')
	plt.title("Position over time")
	plt.grid(True)

	# plot attitude data
	plt.subplot(1,2,2)
	plt.plot(time, desired_att[:,0])
	plt.plot(time, desired_att[:,1])
	plt.plot(time, desired_att[:,2])
	plt.plot(time, actual_att[:,0])
	plt.plot(time, actual_att[:,1])
	plt.plot(time, actual_att[:,2])	

	plt.xlabel('Sim time (s)')
	plt.ylabel('Degrees (deg)')
	plt.title("Attitude over time")
	plt.grid(True)

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
df = df.drop([0])

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
time = df["Time"].astype("float")
time = np.array(df["Time"].tolist())
desired_pos = np.array(df["Desired Position"].values.tolist())
actual_pos = np.array(df["Actual Position"].values.tolist())
desired_att = np.array(df["Desired Attitude"].values.tolist())
actual_att = np.array(df["Actual Attitude"].values.tolist())

# print(type(time[0]))
# print(type(desired_pos[0]))
# print(type(actual_pos[0]))
# print(type(desired_att[0]))
# print(type(actual_att[0]))

visualize_results()