// Quadcopter geometric controller member function definitions
//
// By: Patrick Ledzian
// Date: 12 November 2020
//

#include "../../include/geometric_include/quadcopter.hpp"

// Controller class initializations and definitions
Quadcopter::Controller::Controller()
    : Kpos_(16.0),                         // controller gains
      Kvel_(5.6),
      Krot_(8.81),
      Kang_(2.54),
      pos_err_(),                          // pos/vel/rot/angvel errors
      vel_err_(),
      rot_err_(),
      ang_vel_err_(),
      pos_time_(0.0),                      // time-keeping
      att_time_(0.0),
      pos_time_loop_(0.01),
      att_time_loop_(0.001),
      Rc_(),                               // "computed" values
      Rc_prev_(),
      omegac_(),
      omegac_prev_(),
      desired_rotor_forces_(),             // N?
      desired_rotor_rates_(),              // rad/s
      desired_thrust_magnitude_(),         // sum of rotor forces
      desired_moments_()                   // output from attitude controller
{
    ;    // don't forget the : initializers
}

//// Calculate geometric position control
void Quadcopter::Controller::position_control(Quadcopter &q)
{
    // only run loop if enough time has passed
    if ((q.sim_time_ - pos_time_) > pos_time_loop_)
    {
        // for position controlled flight mode
        Eigen::Matrix<double, 1, 3> e3_basis;

        e3_basis << 0.0, 0.0, 1.0;
        pos_err_ = q.sensor_pos_ - q.desired_pos_;
        vel_err_ = q.derived_lin_vel_ - q.desired_vel_;

        desired_thrust_magnitude_ =
                (Kpos_ * pos_err_ + Kvel_ * vel_err_ + q.mass_ * q.gravity_ * e3_basis - q.mass_ * q.desired_acc_) *
                q.derived_rot_ * e3_basis.transpose();

        pos_time_ = q.sim_time_;                // should be the last thing run
    } // end pos timing check
} // end Quadcopter::Controller:position_control()

//// Calculate geometric attitude control
void Quadcopter::Controller::attitude_control(Quadcopter &q)
{
    // only run loop if enough time has passed
    if((q.sim_time_ - att_time_) > att_time_loop_)
    {
        // TODO: build out the geometric attitude controller

        // ORDER OF OPERATIONS
        // 1) calculate computed basis vectors b1c, b2c, b3c
        // 2) calculate computed attitude Rc
        // 3) calculate derivative of the computed attitude Rc
        // 4) calculate computed angular velocity omegac
        // 5) calculate rotation error and angular velocity error
        // 6) calculate desired moments
        // 7) calculate desired rotor rates
        // 8) set rotor rates (not publishing yet)
        // 9) set "memory" variables for time derivations

        // 1) calculate computed basis vectors b1c, b2c, b3c
        Eigen::Matrix<double, 1, 3> b1c;
        Eigen::Matrix<double, 1, 3> b2c;
        Eigen::Matrix<double, 1, 3> b3c;
        Eigen::Matrix<double, 1, 3> e3_basis;
        Eigen::Matrix<double, 1, 3> temp_vec;
        Eigen::Matrix<double, 3, 3> temp_mat;

        e3_basis << 0.0, 0.0, 1.0;
        temp_vec = -Kpos_ * pos_err_ - Kvel_ * vel_err_ - q.mass_ * q.gravity_ * e3_basis + q.mass_ * q.desired_acc_;

        b3c = temp_vec / temp_vec.squaredNorm();
        b1c = -(1.0 / (b3c.cross(q.desired_pos_)).squaredNorm()) * b3c.cross((b3c.cross(q.desired_pos_)));
        b2c = b3c.cross(b1c);

        // 2) calculate computed attitude Rc
        Rc_ << b1c(0), b2c(0), b3c(0),
                b1c(1), b2c(1), b3c(1),
                b1c(2), b2c(2), b3c(2);

        // 3) calculated derivative of the computed attitude Rc
        Eigen::Matrix<double, 3, 3> Rc_dot;
        Rc_dot = (Rc_ - Rc_prev_) / (q.sim_time_ - att_time_);

        // 4) calculate computed angular velocity omegac
        Eigen::Matrix<double, 3, 3> omegac_hat;
        omegac_hat = Rc_.transpose() * Rc_dot;
        omegac_(0) = omegac_hat(1, 2);
        omegac_(1) = omegac_hat(0, 2);
        omegac_(2) = omegac_hat(0, 1);

        // 5) calculate rotation error and angular velocity error
        temp_mat = Rc_.transpose() * q.derived_rot_ - q.derived_rot_.transpose() * Rc_;
        rot_err_(0) = -temp_mat(1, 2);
        rot_err_(1) = temp_mat(0, 2);
        rot_err_(2) = -temp_mat(0, 1);
        rot_err_ = 0.5 * rot_err_;

        ang_vel_err_ = q.derived_pqr_att_ - (q.derived_rot_.transpose() * Rc_ * omegac_.transpose()).transpose();

        // 6) calculate desired moments
        Eigen::Matrix<double, 3, 3> omega_hat;

        // TODO: implement "hat" property checks
        omega_hat << 0.0, -q.derived_pqr_att_(2), q.derived_pqr_att_(1),
                q.derived_pqr_att_(2), 0.0, -q.derived_pqr_att_(0),
                -q.derived_pqr_att_(1), q.derived_pqr_att_(0), 0.0;

        Eigen::Matrix<double, 1, 3> omegac_dot;
        omegac_dot = (omegac_ - omegac_prev_) / (q.sim_time_ - att_time_);

        desired_moments_ = -Krot_ * rot_err_ - Kang_ * ang_vel_err_ +
                           q.derived_pqr_att_.cross((q.J_ * q.derived_pqr_att_.transpose()).transpose())
                           - (q.J_ * (omega_hat * q.derived_rot_.transpose() * Rc_ * omegac_.transpose() -
                                      q.derived_rot_.transpose() * Rc_ * omegac_dot.transpose())).transpose();

        // 7) calculate desired rotor rates
        Eigen::Matrix<double, 4, 1> temp;
        temp(0) = desired_thrust_magnitude_;
        temp(1) = desired_moments_(0);
        temp(2) = desired_moments_(1);
        temp(3) = desired_moments_(2);

        desired_rotor_forces_ = q.inv_motor_force_mapping_ * temp;
        desired_rotor_rates_ = (desired_rotor_forces_ / q.motor_force_const_).cwiseSqrt();  // uses F = kf \omega^{2}

        // 8) set rotor rates (not publishing yet)
        set_rotor_rates(q);

        // 9) set "memory" variables for time derivations
        Rc_prev_ = Rc_;
        omegac_prev_ = omegac_;
        att_time_ = q.sim_time_;            // should be the last thing run
    }
} // end Quadcopter::Controller_attitude_control()

//// Set the rotor rates of the pass quadcopter object
void Quadcopter::Controller::set_rotor_rates(Quadcopter &q)
{
    q.ref_motor_vel0_.set_data(desired_rotor_rates_(0));
    q.ref_motor_vel1_.set_data(desired_rotor_rates_(1));
    q.ref_motor_vel2_.set_data(desired_rotor_rates_(2));
    q.ref_motor_vel3_.set_data(desired_rotor_rates_(3));

} // end Quadcopter::Controller::set_rotor_rates()