# quadcopter_simulation_gazebo
Includes a motor model that is applied to each motor in the iris sdf file. Publish->Subscribe communications.


How to build:
1) clone the repo
2) move the "iris_test" folder into "~/.gazebo/models/"; this is your vehicle model and world model
3) in "quadcopter_simulation_gazebo" run `mkdir build` and `cd build`
4) link your new libraries `export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:<full path to dir>/build`
5) from "build" run `cmake ../` then `make`
6) run the simulation from the build folder `gazebo --verbose ../iris_testing.world`
7) in a separate terminal run one of the following commands
	- `./hover_env_control hover`; take-off and hover at 2m
	- `./hover_env_control land`; land via open-loop. 
	- `./hover_env_control steps`; take-off, hover, and execute a series of step responses
	- `./hover_env_control min_snap`; take-off and fly an S-shaped trajectory generated via optimizing for minimum snap (smooth acceleration)
	- `./hover_env_control circle`; take-off and fly a circling trajectory
	- `./hover_env_control fig8`; take-off and fly a figure-8 trajectory

8) For analysis and visualization, move the file "test_data.txt" from "build" to the path "iris_platform_analysis/sim_test_data" and in "iris_platform_analysis" run `python3 pos_att_analysis.py`


![demo](vids/min_snap_fast.gif)
