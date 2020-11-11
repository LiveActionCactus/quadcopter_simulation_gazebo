// Header file for geometric_control.cpp
//
// By: Patrick Ledzian
// Date: 09 November 2020
//

#ifndef GAZEBOVEHICLESIMBINDINGS_GEOMETRIC_CONTROL_H
#define GAZEBOVEHICLESIMBINDINGS_GEOMETRIC_CONTROL_H

#include "quadcopter.h"
#include "trajectory.h"

class Controller: Trajectory{
public:
    Controller();
    // different position and attitude controllers (could create position and attitude subclasses)

private:
    // controller specific data/properties; bounds checking (geometric modes; ctrl gains)
    //// controller gains
    static Eigen::Matrix<double,1,3> Kp_pos_;
    static Eigen::Matrix<double,1,3> Kd_pos_;
    static Eigen::Matrix<double,1,3> Kp_ang_;
    static Eigen::Matrix<double,1,3> Kd_ang_;
};


#endif //GAZEBOVEHICLESIMBINDINGS_GEOMETRIC_CONTROL_H
