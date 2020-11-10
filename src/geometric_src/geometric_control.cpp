//
// Created by odysseus on 11/8/20.
//

#include "../../include/geometric_include/geometric_control.h"
#include <iostream>

// Quadcopter class initializations and definitions
Quadcopter::Quadcopter()
:   sim_time_(0.0),
    prev_sim_time_(0.0),
    sim_state_(0),
    ref_motor_vel0_(),                   // default constructed Float protobuf messages
    ref_motor_vel1_(),
    ref_motor_vel2_(),
    ref_motor_vel3_(),
    gravity_(9.81),                     // m/s^2
    mass_(1.5),                         // kg
    hover_point_(665.0),                // rad/s for one motor only
    motor_force_const_(8.54858e-06),
    motor_mapping_(),                   // default constructed Eigen matrices (all zeros)
    sensor_pos_(),                      // m
    prev_sensor_pos_(),
    derived_lin_vel_(),                 // m/s
    sensor_quat_(),
    derived_euler_att_(),               // rad
    prev_derived_euler_att_(),
    derived_pqr_att_(),                 // rad/s
    desired_pos_(),                     // m
    desired_vel_(),                     // m/s
    desired_acc_(),                     // m/s^{2}
    desired_euler_att_(),               // rad
    desired_pqr_att_(),                 // rad/s
    desired_thrust_(),                  // rad/s
    q_hat_(),
    quat_normalized_(),
    final_att_deltas_()
{
    // don't forget the : initializers
    motor_mapping_ << 1.0,  0.5,  0.5,  1.0,
                      1.0, -0.5,  0.5, -1.0,
                      1.0, -0.5, -0.5,  1.0,
                      1.0,  0.5, -0.5, -1.0;
    sensor_quat_ << 1.0, 0.0, 0.0, 0.0;
}

//void Quadcopter::update_sensor_data(ConstLocalPosesStampedPtr &local_pose)
//{
//    sim_time_ = local_pose->time().sec() + (local_pose->time().nsec() * 10E-10);
//
//    sensor_pos_(0) = local_pose->pose(0).position().x();
//    sensor_pos_(1) = local_pose->pose(0).position().y();
//    sensor_pos_(2) = local_pose->pose(0).position().z();
//
//    sensor_quat_(0) = local_pose->pose(0).orientation().w();
//    sensor_quat_(1) = local_pose->pose(0).orientation().x();
//    sensor_quat_(2) = local_pose->pose(0).orientation().y();
//    sensor_quat_(3) = local_pose->pose(0).orientation().z();
//}

// Trajectory class initializations and definitions
Trajectory::Trajectory()
{
    // don't forget the : initializers
}

// Controller class initializations and definitions
Controller::Controller()
{
    // don't forget the : initializers
}

// TODO: replace this and "local_poses_cb" with attributes and a member function in the Quadcopter class
// Subscribe to measured position and orientation values
//gazebo::transport::SubscriberPtr sub = _node_handle->Subscribe("~/pose/local/info", local_poses_cb);


// TODO: solve the static variable issue
//void Quadcopter::local_poses_cb(ConstLocalPosesStampedPtr &local_pose)
//{
////    quad.update_sensor_data(local_pose);
//;
//} // end local_poses_cb

// MAIN-LOOP (should have very few lines)
int main()
{
    std::cout << "Test Geometric Control" << std::endl;

    Quadcopter test;
}