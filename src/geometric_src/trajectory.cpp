// Trajectory member function definitions
//
// By: Patrick Ledzian
// Date: 11 November 2020
//

#include <iostream>
#include "../../include/geometric_include/quadcopter.hpp"

//// Trajectory class initializations and definitions
Quadcopter::Trajectory::Trajectory(std::string traj_)
    :   hover_("hover"),            // command line arguments to run trajectory
        steps_("step"),
        min_snap_("min_snap"),
        circle_("circ"),
        figure_eight_("fig8"),
        set_pt1_(1),                // 1 -- active; 0 -- inactive
        set_pt2_(0),
        set_pt3_(0),
        set_pt4_(0),
        desired_traj_(traj_),
        desired_pos_(),             // 1x3 eigen matrices
        desired_vel_(),
        desired_acc_(),
        desired_euler_att_(),
        desired_pqr_att_()          // 1x4 eigen matrix
{
    // don't forget the : initializers
    ;
} // end Quadcopter::Trajectory::Trajectory()

//// Run trajectory loop in forward direction
void Quadcopter::Trajectory::run_trajectory_update()
{
    if(desired_traj_ == "none"){
//        std::cout << "constructor ran; now inside traj update" << std::endl;
        return;
    } else if(desired_traj_ == hover_){
        basic_hover();
        return;
    } else if(desired_traj_ == steps_){
        ;
    } else if(desired_traj_ == min_snap_){
        ;
    } else if(desired_traj_ == circle_){
        ;
    }else if(desired_traj_ == figure_eight_){
        ;
    }else{
        ; // TODO: throw an exception; we should never end up here
    }

} // end Quadcopter::Trajectory::run_trajectory_update()

//// Change the desired trajectory
void Quadcopter::Trajectory::set_new_trajectory(std::string new_traj_)
{
    if (new_traj_ == "none" || new_traj_ ==  hover_ || new_traj_ == steps_ || new_traj_ == min_snap_
            || new_traj_ == circle_ || new_traj_ == figure_eight_) {
        desired_traj_ = new_traj_;
    } else {
        std::cout << "Invalid trajectory; change not made" << std::endl;
    }

} // end Quadcopter::Trajectory::set_new_trajectory()

//// Basic takeoff and hover
void Quadcopter::Trajectory::basic_hover()
{
    // gets the quadcopter hovering at a stable state
    desired_pos_ << 0.0, 0.0, 2.0;
    std::cout << "i'm setting hover" << std::endl;

} // end Quadcopter::Trajectory::basic_hover()