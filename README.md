# quadcopter_simulation_gazebo
Includes a motor model that is applied to each motor in the iris sdf file. Publish->Subscribe communications.


How to build:
1) clone the repo
2) move the "iris_test" folder into "~/.gazebo/models/"; this is your vehicle model and world model
3) in "quadcopter_simulation_gazebo" run `mkdir build` and `cd build`
4) link your new libraries `export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:<full path to dir>/build`
5) from "build" run `cmake ../` then `make`
6) run the simulation from the build folder `gazebo --verbose ../iris_testing.world`
7) in a separate terminal run `./basic_quad_control hover` to take-off in closed loop; run `./basic_quad_control land` to land in open-loop. Step responses with `./basic_quad_control steps` and take-off, hover, and minimum snap trajectory following with `./basic_quad_control min_snap`
8) For analysis and visualization, move the file "test_data.txt" from "build" to the path "iris_platform_analysis/sim_test_data" and in "iris_platform_analysis" run `python3 pos_att_analysis.py`


![demo](vids/min_snap_fast.gif)
