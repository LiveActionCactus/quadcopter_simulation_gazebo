// Header file for trajectory.cpp
//
// By: Patrick Ledzian
// Date: 11 November 2020
//

#ifndef GAZEBOVEHICLESIMBINDINGS_TRAJECTORY_H
#define GAZEBOVEHICLESIMBINDINGS_TRAJECTORY_H

#include "quadcopter.h"

class Trajectory: Quadcopter{
public:
    Trajectory();           // need to specify the trajectory chosen here; can inherit desired position from Quadcopter; need to do some error checking though

private:
    // trajectory specific data; stats; optimization algorithm
    //// different trajectories to choose from
    std::string steps_;
    std::string hover_;
    std::string min_snap_;
    std::string circle_;
    std::string figure_eight_;

    //// setpoints for step response trajectory (maybe put this in "Controller" and have that inherit from Trajectory)
    bool set_pt1_ = 1;
    bool set_pt2_ = 0;
    bool set_pt3_ = 0;
    bool set_pt4_ = 0;

    //// desired values to achieve
    Eigen::Matrix<double,1,3> _desired_pos;
    Eigen::Matrix<double,1,3> _desired_vel;
    Eigen::Matrix<double,1,3> _desired_acc;
    Eigen::Matrix<double,1,3> _desired_euler_att;
    Eigen::Array<double,1,3> _desired_pqr_att;

    //// function definitions
    void basic_hover();
};

#endif //GAZEBOVEHICLESIMBINDINGS_TRAJECTORY_H
