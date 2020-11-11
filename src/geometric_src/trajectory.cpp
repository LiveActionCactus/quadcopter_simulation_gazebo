// Trajectory member function definitions
//
// By: Patrick Ledzian
// Date: 11 November 2020
//

#include "../../include/geometric_include/trajectory.h"

//// Trajectory class initializations and definitions
Trajectory::Trajectory()
    :   steps_("step"),             // command line arguments to run trajectory
        hover_("hover"),
        min_snap_("min_snap"),
        circle_("circ"),
        figure_eight_("fig8"),
        set_pt1_(1),                // 1 -- active; 0 -- inactive
        set_pt2_(0),
        set_pt3_(0),
        set_pt4_(0),
        _desired_pos(),             // 1x3 eigen matrices
        _desired_vel(),
        _desired_acc(),
        _desired_euler_att(),
        _desired_pqr_att()          // 1x4 eigen matrix
{
    // don't forget the : initializers
    ;
}

//// Basic takeoff and hover
void Trajectory::basic_hover()
{
    // gets the quadcopter hovering at a stable state
    _desired_pos << 0.0, 0.0, 2.0;
}