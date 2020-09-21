# quadcopter_simulation_gazebo
Includes a motor model that is applied to each motor in the iris sdf file. Publish->Subscribe communications.


How to build:
1) clone the repo
2) move the "iris_test" folder into "~/.gazebo/models/"; this is your vehicle model and world model
3) in "quadcopter_simulation_gazebo" run `mkdir build` and `cd build`
4) link your new libraries `export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:<full path to dir>/build`
5) from "build" run `cmake ../` then `make`
6) run the simulation from the build folder `gazebo --verbose ../iris_testing.world`
7) in a separate terminal run `./set_rot_vel takeoff` to take-off in open loop; run `./set_rot_vel land` to land in open-loop
