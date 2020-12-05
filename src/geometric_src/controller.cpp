// Quadcopter geometric controller member function definitions
//
// By: Patrick Ledzian
// Date: 12 November 2020
//

#include "../../include/geometric_include/quadcopter.hpp"

//TODO: Testing
// 1) compare no attitude controller plots to modest attitude controller plots; I'm not sure it's doing anything
//      03Dec: Krot_ 5 -- causes pitch flop, -5 -- roll flop, 0 -- hover then slow drift off
//              points to an issue in the implementation
//      SUCCESS (sort of): no "computed" vector sign changes, just Krot_ = -5 for "okay" hover that slowly goes unstable in roll
// 2) review the math + coding to make sure it's implemented correctly
//      03Dec: +x goes forward like is supposed to but the pitch controller goes unstable (positive feedback)
//             +y goes to the right which is opposite to what it should
//              MAYBE LOOK INTO ALTERING "rot_err_" elements
// 3) it's possible that the position + velocity controller is too high, i'm getting 20N thrust magnitude at start
// 4) try to critically damp the results for hover trajectory (is this even a thing for geometric controllers?)
// 5) are the motors being mapped properly? no issue in the roll control, but pitch is wild...
// !) THE rot_err_ SEEMS TO BE LOOPING VERY SLOWLY (look at the chopping in the plots)
// 7) maybe the timining not being consistent is causing issues with the variable updates (the derived ones)

// Controller class initializations and definitions
Quadcopter::Controller::Controller()
    : Kpos_(25.0),                          // controller gains; 8.0
      Kvel_(12.0),                          // 4.0
      Krot_(-1.5),                         // -1.5             // motors are saturating at 10.0
      Kang_(1.0),                          // 1.0
      pos_err_(),                          // pos/vel/rot/angvel errors
      vel_err_(),
      rot_err_(),
      ang_vel_err_(),
      pos_time_(0.0),                      // time-keeping
      att_time_(0.0),
      pos_time_loop_(0.01),                // 0.01
      att_time_loop_(0.001),               // 0.001
      Rc_(),                               // "computed" values
      Rc_prev_(),
      omegac_(),
      omegac_prev_(),
//      desired_rotor_forces_(),             // N?
      desired_rotor_rates_(),              // rad/s
      desired_thrust_magnitude_(),         // sum of rotor forces
      desired_moments_()                   // output from attitude controller
{
    testdata_controller.open("testdata_cont.txt");
    troubleshooting.open("troubleshooting.txt");
//    desired_rotor_forces_ << 0.0, 0.0, 0.0, 0.0;
//    desired_rotor_rates_ << 0.0, 0.0, 0.0, 0.0;
//    desired_thrust_magnitude_ = 0.0;
//    desired_moments_ << 0.0, 0.0, 0.0;
}

//// Calculate geometric position control
void Quadcopter::Controller::position_control(Quadcopter &q, Trajectory &t)
{
    // only run loop if enough time has passed
    if ((q.sim_time_ - pos_time_) > pos_time_loop_)
    {
        // for position controlled flight mode
        Eigen::Matrix<double, 1, 3> e3_basis;
        Eigen::Matrix<double,1,3> desired_pos_ = t.get_desired_pos();
        Eigen::Matrix<double,1,3> desired_vel_ = t.get_desired_vel();
        Eigen::Matrix<double,1,3> desired_acc_ = t.get_desired_acc();

        e3_basis << 0.0, 0.0, -1.0;          // TODO: 1.0 (orig) or -1.0 (paper)?
        pos_err_ = (q.sensor_pos_ - desired_pos_);
        vel_err_ = (q.derived_lin_vel_ - desired_vel_);

//        // TODO: I flipped these signs to align with gazebo coordinate frame better
//        pos_err_(1) = -pos_err_(1); pos_err_(2) = -pos_err_(2);
//        vel_err_(1) = -vel_err_(1); vel_err_(2) = -vel_err_(2);

        // TODO: it's possible the coordinate frames are wrong, especially with z-axis
        // TODO: maybe just change the signs on the pos/vel error calculations?
        desired_thrust_magnitude_ =
                (Kpos_ * pos_err_ + Kvel_ * vel_err_ + q.mass_ * q.gravity_ * e3_basis - q.mass_ * desired_acc_) *
                q.derived_rot_ * e3_basis.transpose();
//        desired_thrust_magnitude_ = - desired_thrust_magnitude_;

//        std::cout << Kpos_ * pos_err_ << std::endl;
//        std::cout << Kvel_ * vel_err_ << std::endl;
//        std::cout << q.mass_ * q.gravity_ * e3_basis << std::endl;
//        std::cout << q.derived_rot_ << std::endl;
//        std::cout << e3_basis.transpose() << std::endl;
//        std::cout << (q.sim_time_ - pos_time_) << std::endl;
//        std::cout << desired_thrust_magnitude_ << std::endl;

//        std::cout << std::endl;
        pos_time_ = q.sim_time_;                // should be the last thing run

    } // end pos timing check

} // end Quadcopter::Controller:position_control()

//// Calculate geometric attitude control
void Quadcopter::Controller::attitude_control(Quadcopter &q, Trajectory &t)
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
        Eigen::Matrix<double,1,3> desired_pos_ = t.get_desired_pos();
        Eigen::Matrix<double,1,3> desired_vel_ = t.get_desired_vel();
        Eigen::Matrix<double,1,3> desired_acc_ = t.get_desired_acc();
        Eigen::Matrix<double,1,3> b1d;
        b1d << 1.0, 0.0, 0.0;               // how does this tuning parameter behave? how should it be set intelligently?

        e3_basis << 0.0, 0.0, -1.0;
        temp_vec = -Kpos_ * pos_err_ - Kvel_ * vel_err_ - q.mass_ * q.gravity_ * e3_basis + q.mass_ * desired_acc_;

        b3c = temp_vec / temp_vec.squaredNorm();
        b1c = -(1.0 / (b3c.cross(b1d)).squaredNorm()) * b3c.cross((b3c.cross(b1d)));
        b2c = b3c.cross(b1c);

//        // TODO: I flipped these signs to align with gazebo coordinate frame better
//        b2c = -b2c;
//        b3c = -b3c;       // TODO: the position and velocity errors are defined in pos control

        // TODO: I changed the signs on b2c and b3c to negative to try to better align the frames
        // 2) calculate computed attitude Rc
//        Rc_ <<  b1c(0), b2c(0), b3c(0),   // most likely (column vectors)
//                b1c(1), b2c(1), b3c(1),
//                b1c(2), b2c(2), b3c(2);

        Rc_ <<  b1c(0), -b2c(0), -b3c(0),   // most likely (column vectors)
                b1c(1), -b2c(1), -b3c(1),
                b1c(2), -b2c(2), -b3c(2);

//        Rc_ << b1c(0), b1c(1), b1c(2),                    // TODO: testing row vectors
//               b2c(0), b2c(1), b2c(2),
//               b3c(0), b3c(1), b3c(2);

        // 3) calculated derivative of the computed attitude Rc
        Eigen::Matrix<double, 3, 3> Rc_dot;
        Rc_dot = (Rc_ - Rc_prev_) / (q.sim_time_ - att_time_);

        // 4) calculate computed angular velocity omegac
        Eigen::Matrix<double, 3, 3> omegac_hat;
        omegac_hat = Rc_.transpose() * Rc_dot;
        omegac_hat = mat3d_check(-10.0, 10.0, omegac_hat);

        omegac_(0) = omegac_hat(1, 2);
        omegac_(1) = omegac_hat(0, 2);
        omegac_(2) = omegac_hat(0, 1);
        omegac_ = vec3d_check(-10.0, 10.0, omegac_);                // bounds checking

        // 5) calculate rotation error and angular velocity error
        temp_mat = Rc_.transpose() * q.derived_rot_ - q.derived_rot_.transpose() * Rc_;
//        rot_err_(0) = -temp_mat(1, 2);            // TODO: original errors
//        rot_err_(1) = temp_mat(0, 2);
//        rot_err_(2) = -temp_mat(0, 1);

        rot_err_(0) = temp_mat(1, 2);
        rot_err_(1) = temp_mat(0, 2);
        rot_err_(2) = -temp_mat(0, 1);

        rot_err_ = 0.5 * rot_err_;
        rot_err_ = vec3d_check(-2*3.14, 2*3.14, rot_err_);

        ang_vel_err_ = q.derived_pqr_att_ - (q.derived_rot_.transpose() * Rc_ * omegac_.transpose()).transpose();
        ang_vel_err_ = vec3d_check(-10.0, 10.0, ang_vel_err_);          // bounds checking

        // 6) calculate desired moments
        Eigen::Matrix<double, 3, 3> omega_hat;      // needs to be initialized or can cause NAN issues
        omega_hat << 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0;

        // TODO: implement "hat" property checks
        omega_hat << 0.0,                           -q.derived_pqr_att_(2),  q.derived_pqr_att_(1),
                     q.derived_pqr_att_(2),    0.0,                         -q.derived_pqr_att_(0),
                    -q.derived_pqr_att_(1),    q.derived_pqr_att_(0),  0.0;

        Eigen::Matrix<double, 1, 3> omegac_dot;    // TODO: needs to be initialized or can cause NAN issues
        omegac_dot << 0.0, 0.0, 0.0;
        omegac_dot = (omegac_ - omegac_prev_) / (q.sim_time_ - att_time_);
        omegac_dot = vec3d_check(-10.0, 10.0, omegac_dot);              // bounds checking

        desired_moments_ = -Krot_ * rot_err_ - Kang_ * ang_vel_err_ +
                           q.derived_pqr_att_.cross((q.J_ * q.derived_pqr_att_.transpose()).transpose())
                           - (q.J_ * (omega_hat * q.derived_rot_.transpose() * Rc_ * omegac_.transpose() -
                                      q.derived_rot_.transpose() * Rc_ * omegac_dot.transpose())).transpose();

        // TODO: look at individual variables used in the attitude update
        Eigen::Matrix<double,3,3> test_Rc;
        Eigen::Matrix<double,1,3> test_omegacdot;
        Eigen::Matrix<double,3,3> test_derived_rot_;
//        Eigen::Matrix<double,1,3> test_omegac_;
//        Eigen::Matrix<double,3,3> test_J;
//        Eigen::Matrix<double,1,3> test_derived_pqr_att_;
//        Eigen::Matrix<double,1,3> test_ang_vel_err;

        test_Rc = Rc_;
        test_omegacdot = omegac_dot;
        test_derived_rot_ = q.derived_rot_;

        // TODO: troubleshooting spiking behavior
        Eigen::Matrix<double,1,3> test_desired_moments;
        test_desired_moments = desired_moments_;

        desired_moments_ = vec3d_check(-10.0, 10.0, desired_moments_);      // bounds checking

        // 7) calculate desired rotor rates
        Eigen::Matrix<double, 4, 1> temp;
        temp(0) = desired_thrust_magnitude_;
        temp(1) = desired_moments_(0);
        temp(2) = desired_moments_(1);
        temp(3) = desired_moments_(2);

        desired_rotor_rates_ = q.inv_motor_force_mapping_ * temp;
        desired_rotor_rates_ = desired_rotor_rates_.cwiseSqrt();
//        desired_rotor_rates_ = (desired_rotor_forces_ / q.motor_force_const_).cwiseSqrt();  // uses F = kf \omega^{2}
        desired_rotor_rates_= vec4d_check(0.0, 1500.0, desired_rotor_rates_);                 // bounds checking

        // 8) set rotor rates (not publishing yet)
//        set_rotor_rates(q);

        // 9) set "memory" variables for time derivations
        Rc_prev_ = Rc_;
        omegac_prev_ = omegac_;
        att_time_ = q.sim_time_;            // should be the last thing run

//
//    set_rotor_rates(q);                 // needs to be set every loop!

//        testdata_controller << q.sim_time_ << "\n";
//        testdata_controller << std::fixed << std::setprecision(8) << q.sensor_pos_ << "\n";
//        testdata_controller << std::fixed << std::setprecision(8) << desired_pos_ << "\n";
//        testdata_controller << std::fixed << std::setprecision(8) << pos_err_ << "\n";
//        testdata_controller << std::fixed << std::setprecision(8) << omegac_hat << "\n";
//        testdata_controller << std::fixed << std::setprecision(8) << omegac_ << "\n";
//        testdata_controller << std::fixed << std::setprecision(8) << rot_err_ << "\n";
//        testdata_controller << std::fixed << std::setprecision(8) << ang_vel_err_ << "\n";
//        testdata_controller << std::fixed << std::setprecision(8) << omega_hat << "\n";
//        testdata_controller << std::fixed << std::setprecision(8) << omegac_dot << "\n";
//        testdata_controller << std::fixed << std::setprecision(8) << desired_thrust_magnitude_ << "\n";
//        testdata_controller << std::fixed << std::setprecision(8) << desired_moments_ << "\n";
//        testdata_controller << std::fixed << std::setprecision(8) << desired_rotor_rates_ << "\n";
//        testdata_controller  << "\n";

        troubleshooting << q.sim_time_ << "\n";
        troubleshooting << std::fixed << std::setprecision(8) << desired_rotor_rates_ << "\n";
        troubleshooting << std::fixed << std::setprecision(8) << desired_moments_ << "\n";
        troubleshooting << std::fixed << std::setprecision(8) << test_desired_moments << "\n";
        troubleshooting << std::fixed << std::setprecision(8) << test_Rc << "\n";
        troubleshooting << std::fixed << std::setprecision(8) << test_omegacdot << "\n";
        troubleshooting << std::fixed << std::setprecision(8) << test_derived_rot_ << "\n";
        troubleshooting  << "\n";

    } // end att timing check

} // end Quadcopter::Controller_attitude_control()

//// Set the rotor rates of the pass quadcopter object
void Quadcopter::Controller::set_rotor_rates(Quadcopter &q)
{
    q.ref_motor_vel0_.set_data(desired_rotor_rates_(0));
    q.ref_motor_vel1_.set_data(desired_rotor_rates_(1));
    q.ref_motor_vel2_.set_data(desired_rotor_rates_(2));
    q.ref_motor_vel3_.set_data(desired_rotor_rates_(3));

} // end Quadcopter::Controller::set_rotor_rates()

//Eigen::Matrix<double,1,4> Quadcopter::Controller::get_desired_rotor_rates()
//{
//    return(desired_rotor_rates_);
//}

Eigen::Matrix<double,1,3> Quadcopter::Controller::vec3d_check(float _min, float _max, Eigen::Matrix<double,1,3> _vec)
{
    for(int i=0; i<3; i++){
        if(_vec(i) > _max){
            _vec(i) = _max;
        }else if(_vec(i) < _min){
            _vec(i) = _min;
        }else if(isnan(_vec(i))){
            std::cout << "nan!" << std::endl;
            _vec(i) = 0.0;
        }
    } // end for bounds checking

    return(_vec);

} // end vec3d_check

Eigen::Matrix<double,4,1> Quadcopter::Controller::vec4d_check(float _min, float _max, Eigen::Matrix<double,4,1> _vec)
{
    for(int i=0; i<4; i++){
        if(_vec(i) > _max){
            _vec(i) = _max;
        }else if(_vec(i) < _min){
            _vec(i) = _min;
        }else if(isnan(_vec(i))){
            std::cout << "nan!" << std::endl;
            _vec(i) = 0.0;
        }
    } // end for bounds checking

    return(_vec);

} // end vec4d_check

Eigen::Matrix<double,3,3> Quadcopter::Controller::mat3d_check(float _min, float _max, Eigen::Matrix<double,3,3> _mat)
{
    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            if (_mat(i,j) > _max) {
                _mat(i,j) = _max;
            } else if (_mat(i,j) < _min) {
                _mat(i,j) = _min;
            } else if (isnan(_mat(i,j))) {
                std::cout << "nan!" << std::endl;
                _mat(i,j) = 0.0;
            }
        }
    } // end for bounds checking

    return(_mat);

} // end mat3d_check
