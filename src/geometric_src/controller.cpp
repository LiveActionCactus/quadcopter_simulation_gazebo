// Quadcopter geometric controller member function definitions
//
// By: Patrick Ledzian
// Date: 12 November 2020
//

#include "../../include/geometric_include/quadcopter.hpp"

// Controller class initializations and definitions
Quadcopter::Controller::Controller()
    : Kpos_(16.0),
      Kvel_(5.6),
      Krot_(8.81),
      Kang_(2.54),
      desired_rotor_forces_(),             // N?
      desired_rotor_rates_(),              // rad/s
      thrust_magnitude_()                  // sum of rotor forces
{
    ;    // don't forget the : initializers
}

//// Calculate geometric position control
void Quadcopter::Controller::position_control(Quadcopter &q)
{
    // for position controlled flight mode
    Eigen::Matrix<double,1,3> pos_err = q.sensor_pos_ - q.desired_pos_;
    Eigen::Matrix<double,1,3> vel_err = q.derived_lin_vel_ - q.desired_vel_;
    Eigen::Matrix<double,1,3> e3_basis;

    e3_basis << 0.0, 0.0, 1.0;

    thrust_magnitude_ = (Kpos_*pos_err + Kvel_*vel_err + q.mass_*q.gravity_*e3_basis - q.mass_*q.desired_acc_) * q.derived_rot_ * e3_basis.transpose();


} // end Quadcopter::Controller:position_control()

//// Calculate geometric attitude control
void Quadcopter::Controller::attitude_control(Quadcopter &q)
{
    // TODO: build out the geometric attitude controller

    // TODO: covert the desired rotor forces into rates
    set_rotor_rates(q);

} // end Quadcopter::Controller_attitude_control()

//// Set the rotor rates of the pass quadcopter object
void Quadcopter::Controller::set_rotor_rates(Quadcopter &q)
{
    q.ref_motor_vel0_.set_data(desired_rotor_rates_(0));
    q.ref_motor_vel1_.set_data(desired_rotor_rates_(1));
    q.ref_motor_vel2_.set_data(desired_rotor_rates_(2));
    q.ref_motor_vel3_.set_data(desired_rotor_rates_(3));

} // end Quadcopter::Controller::set_rotor_rates()