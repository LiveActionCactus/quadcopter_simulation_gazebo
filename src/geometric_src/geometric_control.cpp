// Geometric control of a quadcopter in the Gazebo simulation environment
//
// By: Patrick Ledzian
// Date: 09 November 2020
//

#include "../../include/geometric_include/geometric_control.h"
#include <iostream>

// troubleshooting: cmake -DCMAKE_BUILD_TYPE=Debug .. // gdb ./geometric_control // r // bt --- should break automatically on errors

// Controller class initializations and definitions
Controller::Controller()
{
    // don't forget the : initializers
}

// MAIN-LOOP (should have very few lines)
int main(int _argc, char **_argv)
{
    // Program Setup
    std::cout << "Connecting to Gazebo client..." << std::endl;
    gazebo::client::setup(_argc, _argv);

    std::cout << "Building Quadcopter objects..." << std::endl;
    Quadcopter test;

    std::cout << "Running..." << std::endl;
    std::cout << std::endl;

    // Run
    while(1)
    {
        test.update_state_and_data();
    }

    gazebo::client::shutdown();         // this has to be at the end of main -- since we are spawning subscriber cb threads in the objects
}