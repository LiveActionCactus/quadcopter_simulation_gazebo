# Perform analysis of a motor to generate thrust and torque mappings
# 
# By: Patrick Ledzian
# Created: 02 September 2020

import numpy as np
import matplotlib.pyplot as plt

thrust_const = 8.54858e-06
torque_const = 0.016
vehicle_mass = 1.5
hover_force = 1.5 / .101971621297793 	# Kg -> Newtons


def thrust_analysis(th_const, viz=False, verbose=False):
	rot_spd_vals = np.linspace(0, 1000, 1000) 						# rad/s
	forces = np.multiply(np.square(rot_spd_vals), th_const) 		# F = omega^2 * th_const  (in Newtons)
	
	hover_rate_idx = min(range(len(forces)), key=lambda i: abs(forces[i]-(hover_force/4)))
	hover_rate = rot_spd_vals[hover_rate_idx]
	print("Total force needed for hover: ", round(hover_force, 1), "N")
	print("Individual rotor speed for hover: ", round(hover_rate, 1), "rad/s")

	if verbose:
		print("Rotor speeds: ", rot_spd_vals)
		print("Forces: ", forces)
	
	if viz:
		vizualize_thrust_curve(forces, rot_spd_vals, th_const)

def vizualize_thrust_curve(F, omega, k_f):
	fig, ax = plt.subplots()
	ref_curve = ax.plot(omega, F)
	
	ax.set_xlabel('Rotor speed (rad/s)')
	ax.set_ylabel('Thrust (?)')
	ax.set_title("Rotor speed vs. thrust")
	# ax.grid(True)
	plt.show()

thrust_analysis(thrust_const, viz=True, verbose=False)