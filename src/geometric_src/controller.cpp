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
      pos_err_(),
      vel_err_(),
      desired_rotor_forces_(),             // N?
      desired_rotor_rates_(),              // rad/s
      desired_thrust_magnitude_(),         // sum of rotor forces
      desired_moments_()
{
    ;    // don't forget the : initializers
}

//// Calculate geometric position control
void Quadcopter::Controller::position_control(Quadcopter &q)
{
    // for position controlled flight mode
    Eigen::Matrix<double,1,3> e3_basis;

    e3_basis << 0.0, 0.0, 1.0;
    pos_err_ = q.sensor_pos_ - q.desired_pos_;
    vel_err_ = q.derived_lin_vel_ - q.desired_vel_;


    desired_thrust_magnitude_ = (Kpos_*pos_err_ + Kvel_*vel_err_ + q.mass_*q.gravity_*e3_basis - q.mass_*q.desired_acc_) * q.derived_rot_ * e3_basis.transpose();


} // end Quadcopter::Controller:position_control()

//// Calculate geometric attitude control
void Quadcopter::Controller::attitude_control(Quadcopter &q)
{
    // TODO: build out the geometric attitude controller
    Eigen::Matrix<double,3,3> Rc;
    Eigen::Matrix<double,1,3> b1c;
    Eigen::Matrix<double,1,3> b2c;
    Eigen::Matrix<double,1,3> b3c;
    Eigen::Matrix<double,1,3> e3_basis;
    Eigen::Matrix<double,1,3> temp;
    Eigen::Matrix<double,1,3> omega_hat;

    // TODO: implement "hat" property checks
    omega_hat <<  0.0,                         -q.derived_pqr_att_(2),   q.derived_pqr_att_(1),
                  q.derived_pqr_att_(2),  0.0,                          -q.derived_pqr_att_(0),
                 -q.derived_pqr_att_(1),  q.derived_pqr_att_(0),   0.0;
    e3_basis << 0.0, 0.0, 1.0;

    temp = -Kpos_*pos_err_ - Kvel_*vel_err_ - q.mass_*q.gravity_*e3_basis + q.mass_*q.desired_acc_;
    b3c = temp / temp.squaredNorm();
    b1c = -(1.0 / (b3c.cross(q.desired_pos_)).squaredNorm()) * b3c.cross((b3c.cross(q.desired_pos_)));
    b2c = b3c.cross(b1c);

    Rc << b1c(0), b2c(0), b3c(0),
          b1c(1), b2c(1), b3c(1),
          b1c(2), b2c(2), b3c(2);

    // TODO: need \dot{Rc}, \dot{\Omega c}, vee map from the hatted omega
    // TODO: need the time slice for attitude update

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