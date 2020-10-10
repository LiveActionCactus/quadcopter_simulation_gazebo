# Testing the minimum snap results from Eigen and NumPy 


def minimun_snap_trajectory(self, cur_pos, cur_time):
	if cur_time == 0:
		global path
		global total_time
		global ts
		global coef
		path = np.array([[cur_pos[0], cur_pos[0],       cur_pos[0] + 1.0, cur_pos[0] + 1.0, cur_pos[0],       cur_pos[0]],
						 [cur_pos[1], cur_pos[1],       cur_pos[1],       cur_pos[1] + 1.0, cur_pos[1] + 1.0, cur_pos[1]],
						 [cur_pos[2], cur_pos[2] + 2.0, cur_pos[2] + 2.0, cur_pos[2] + 2.0, cur_pos[2] + 2.0, cur_pos[2] + 2.0]
						 ])
		path = path[:, 0:3] 			# TODO: remove after troubleshooting
		# TODO: do here, "plot_path()" function for ideal path
		(ts, total_time) = self.generate_ts(path)						# create time expectation for each waypoint
		# Coef = self.trajectory_optimization(path) 		# solve for optimal polynomial coefficients
		coef = self.trajectory_optimization(np.transpose(path)) 		# transpose the input path...
		print("RUNNING TRAJ GEN, SHOULD HAPPEN ONCE")
		print("Starting position: ", cur_pos)
		print("First wpt: ", path[:, 0])
		print("Second wpt: ", path[:, 1])
		print("last wpt: ", path[:, -1])
		print("Time series: ", ts)
		print("Total time: ", total_time)
		# TODO: do I need a return statement here? my code doesn't have an initialization loop....
		# return np.array([cur_pos[0], cur_pos[1], cur_pos[2], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

	# cur_time = 0.0						# maybe useful debugging... DO NOT INCLUDE THIS TERM
	if cur_time >= total_time:
		pos = path[:, -1] 					# TODO: tied this to the last achieved position, if the simulation length isn't long enough it'll jump, maybe have adaptive length/duration simulation
		vel = [0.0, 0.0, 0.0]
		acc = [0.0, 0.0, 0.0]
		# TODO: main reason for the jump right now is that there are no x-y values coming out of the optimizer
	else:
		k = (ts <= cur_time).nonzero()
		k = k[-1][-1]						# TODO: maybe because of a sign error the path never iterates wpts

		# THESE CALCS ARE OKAY
		pos_temp = np.array([cur_time**7, cur_time**6, cur_time**5, cur_time**4, cur_time**3, cur_time**2, cur_time, 1])
		pos = np.matmul(pos_temp, coef[8*k:8*(k+1), :])
		# print("pos coef: ", coef[8*k:8*(k+1), :])
		vel_temp = np.array([7*cur_time**6, 6*cur_time**5, 5*cur_time**4, 4*cur_time**3, 3*cur_time**2, 2*cur_time, 1, 0])
		vel = np.matmul(vel_temp, coef[8*k:8*(k+1), :])
		# print("vel coef: ", coef[8*k:8*(k+1), :])
		acc_temp = np.array([42*cur_time**5, 30*cur_time**4, 20*cur_time**3, 12*cur_time**2, 6*cur_time, 2, 0, 0])
		acc = np.matmul(acc_temp, coef[8*k:8*(k+1), :])
		# print(pos)
		# print(vel)
		# print(acc)
		# print(cur_pos)
		# print(path)
		# exit()
		# input()

	new_des_state = np.concatenate((pos, vel, acc))

	return new_des_state


def generate_ts(path):
	speed = 0.5
	path_len = np.sum(np.sqrt(np.sum(np.power(path[:, 1:] - path[:, 0:-1], 2), 0)))
	total_time = path_len/speed
	path_seg_len = np.sqrt(np.sqrt(np.sum(np.power(path[:, 1:] - path[:, 0:-1], 2), 0)))
	ts = np.cumsum(path_seg_len)
	ts = ts/ts[-1]
	ts = np.insert(ts, 0, 0)
	ts = ts*total_time

	return ts, total_time


def trajectory_optimization(path):
	#
	# path: 		m+1 x 3 array of desired waypoints
	#
	# X: 			8m x 3 array of optimal coeficients
	#					x_k(t)=X(8*k:8*(k+1))'*[t^3;t^2,t,1]
	# 					where d \in [1,2,3] to indicate dimension xyz

	# TODO: fix the coefficient generation to include x and y components
	path0 = path
	shape = path0.shape
	m = shape[0]
	n = shape[1]
	m = m-1 			# mathematical convenience
	eps = 2e-52 			# TODO: what is the purpose of this?, eps = 2e-52
							# WHEN THIS IS NOT 2E-52 THE Z COMPONENT GOES HAYWIRE, not sure why probably numerical issues
	# In a 7th order (minimum snap) trajectory optimization there are 8 parameters for each subpath
	X = np.zeros((8*m, n))
	A = np.zeros((8*m, 8*m, n))
	Y = np.zeros((8*m, n))

	# TODO: [95% certain this is fixed] verify that n+1 is needed and not n (NEED 0 INDEX IN THESE) (Python doesn't count last index, MATLAB does)
	for i in range(0, n):
		A[:, :, i] = np.eye(8*m) * eps 			# prevents inversions from hitting singularity

		# Constraint 1: x_k(t_k) = x_{k+1}(t_k) = p_k, where p_k is a waypoint
		idx = 0 					# constraint counter
		# TODO: [95% certain this is fixed] verify that m is needed and not m-1
		for k in range(0, m-1):
			A[idx, 8*k:8*(k+1), i] = np.array([ts[k+1]**7, ts[k+1]**6, ts[k+1]**5, ts[k+1]**4, ts[k+1]**3, ts[k+1]**2, ts[k+1], 1.0]) 			# 1:8, 9:16, 17:24; want 0:7, 8:15, 16:23
			Y[idx, i] = path0[k+1, i]
			idx += 1
			A[idx, 8*(k+1):8*(k+2), i] = np.array([ts[k+1]**7, ts[k+1]**6, ts[k+1]**5, ts[k+1]**4, ts[k+1]**3, ts[k+1]**2, ts[k+1], 1]) 		# 9:16, 17:24; want 8:15, 16:23
			Y[idx, i] = path0[k+1, i]
			idx += 1

		# Constraint 2: \dot{x}_k(t_k) = \dot{x}_{k+1}(t_k)
		for k in range(0, m-1):
			A[idx, 8*k:8*(k+1), i] = np.array([7*ts[k+1]**6, 6*ts[k+1]**5, 5*ts[k+1]**4, 4*ts[k+1]**3, 3*ts[k+1]**2, 2*ts[k+1], 1, 0])
			A[idx, 8*(k+1):8*(k+2), i] = np.array([-7*ts[k+1]**6, -6*ts[k+1]**5, -5*ts[k+1]**4, -4*ts[k+1]**3, -3*ts[k+1]**2, -2*ts[k+1], -1, 0]) 		# 9:16, 17:24; want 8:15, 16:23
			Y[idx, i] = 0
			idx += 1

		# Constraint 3: \ddot{x}_k(t_k) = \ddot{x}_{k+1}(t_k)
		for k in range(0, m-1):
			A[idx, 8*k:8*(k+1), i] = [42*ts[k+1]**5, 30*ts[k+1]**4, 20*ts[k+1]**3, 12*ts[k+1]**2, 6*ts[k+1], 2, 0, 0]
			A[idx, 8*(k+1):8*(k+2), i] = [-42*ts[k+1]**5, -30*ts[k+1]**4, -20*ts[k+1]**3, -12*ts[k+1]**2, -6*ts[k+1], -2, 0, 0]
			Y[idx, i] = 0
			idx += 1

		# Constraint 4: x^(3)_k(t_k) = x^(3)_{k+1}(t_k)
		for k in range(0, m-1):
			A[idx, 8*k:8*(k+1), i] = [210*ts[k+1]**4, 120*ts[k+1]**3, 60*ts[k+1]**2, 24*ts[k+1], 6, 0, 0, 0]
			A[idx, 8*(k+1):8*(k+2), i] = [-210*ts[k+1]**4, -120*ts[k+1]**3, -60*ts[k+1]**2, -24*ts[k+1], -6, 0, 0, 0]
			Y[idx, i] = 0
			idx += 1

		# Constraint 5: x^(4)_k(t_k) = x^(4)_{k+1}(t_k)
		for k in range(0, m-1):
			A[idx, 8*k:8*(k+1), i] = [840*ts[k+1]**3, 360*ts[k+1]**2, 120*ts[k+1], 24, 0, 0, 0, 0]
			A[idx, 8*(k+1):8*(k+2), i] = [-840*ts[k+1]**3, -360*ts[k+1]**2, -120*ts[k+1], -24, 0, 0, 0, 0]
			Y[idx, i] = 0
			idx += 1

		# Constraint 6: x^(5)_k(t_k) = x^(5)_{k+1}(t_k)
		for k in range(0, m-1):
			A[idx, 8*k:8*(k+1), i] = [2520*ts[k+1]**2, 720*ts[k+1], 120, 0, 0, 0, 0, 0]
			A[idx, 8*(k+1):8*(k+2), i] = [-2520*ts[k+1]**2, -720*ts[k+1], -120, 0, 0, 0, 0, 0]
			Y[idx, i] = 0
			idx += 1

		# Constraint 7: x^(6)_k(t_k) = x^(6)_{k+1}(t_k)
		for k in range(0, m-1):
			A[idx, 8*k:8*(k+1), i] = [5040*ts[k+1], 720, 0, 0, 0, 0, 0, 0]
			A[idx, 8*(k+1):8*(k+2), i] = [-5040*ts[k+1], -720, 0, 0, 0, 0, 0, 0]
			Y[idx, i] = 0
			idx += 1

		# so far there are 8(m-1) constraints
		# there are 8 left:
		#    x_1(t_0) = p_0
		#    x^(1)_0(t_0) = 0
		#    x^(2)_0(t_0) = 0
		#    x^(3)_0(t_0) = 0
		#    x_T(t_T) = p_T
		#    x^(1)_T(t_T) = 0
		#    x^(2)_T(t_T) = 0
		#    x^(3)_T(t_T) = 0

		k = 0
		A[idx, 8*k:8*(k+1), i] = [ts[k]**7, ts[k]**6, ts[k]**5, ts[k]**4, ts[k]**3, ts[k]**2, ts[k], 1]
		Y[idx, i] = path0[k, i]
		idx += 1
		A[idx, 8*k:8*(k+1), i] = [7*ts[k]**6, 6*ts[k]**5, 5*ts[k]**4, 4*ts[k]**3, 3*ts[k]**2, 2*ts[k], 1, 0]
		Y[idx, i] = 0
		idx += 1
		A[idx, 8*k:8*(k+1), i] = [42*ts[k]**5, 30*ts[k]**4, 20*ts[k]**3, 12*ts[k]**2, 6*ts[k], 2, 0, 0]
		Y[idx, i] = 0
		idx += 1
		A[idx, 8*k:8*(k+1), i] = [210*ts[k]**4, 120*ts[k]**3, 60*ts[k]**2, 24*ts[k], 6, 0, 0, 0]
		Y[idx, i] = 0
		idx += 1

		# TODO: [95% certain this is correct] double check that this should be "m-1" and not m
		k = m-1
		A[idx, 8*k:8*(k+1), i] = [ts[k+1]**7, ts[k+1]**6, ts[k+1]**5, ts[k+1]**4, ts[k+1]**3, ts[k+1]**2, ts[k+1], 1]
		Y[idx, i] = path0[k+1, i]
		idx += 1
		A[idx, 8*k:8*(k+1), i] = [7*ts[k+1]**6, 6*ts[k+1]**5, 5*ts[k+1]**4, 4*ts[k+1]**3, 3*ts[k+1]**2, 2*ts[k+1], 1, 0]
		Y[idx, i] = 0
		idx += 1
		A[idx, 8*k:8*(k+1), i] = [42*ts[k+1]**5, 30*ts[k+1]**4, 20*ts[k+1]**3, 12*ts[k+1]**2, 6*ts[k+1], 2, 0, 0]
		Y[idx, i] = 0
		idx += 1
		A[idx, 8*k:8*(k+1), i] = [210*ts[k+1]**4, 120*ts[k+1]**3, 60*ts[k+1]**2, 24*ts[k+1], 6, 0, 0, 0]
		Y[idx, i] = 0
		idx += 1

		X[:, i] = np.linalg.solve(A[:, :, i], Y[:, i])

	return X

path = np.array([
	[0.0, 0.0, 2.0],
	[1.0, 0.0, 2.0],
	[1.0, 1.0, 2.0],
	[0.0, 1.0, 2.0],
	[-1.0, 1.0, 2.0],
	[-1.0, 1.0, 2.0]
	])