#include "include/basic_quad_control.hpp"
//#include "include/rotor_plugin.hpp"

/////////////////////////////////////////////////
///

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"

//
// Callback functions providing measured motor velocity
//
void rotor0_cb(MotorSpeedPtr &rotor_vel)
{
    sensor_motor_vel0 = static_cast<double>(rotor_vel->data());
}

void rotor1_cb(MotorSpeedPtr &rotor_vel)
{
    sensor_motor_vel1 = static_cast<double>(rotor_vel->data());
}

void rotor2_cb(MotorSpeedPtr &rotor_vel)
{
    sensor_motor_vel2 = static_cast<double>(rotor_vel->data());
}

void rotor3_cb(MotorSpeedPtr &rotor_vel)
{
    sensor_motor_vel3 = static_cast<double>(rotor_vel->data());
}

// TODO: not sure if "iris" is the same index every run, need to run a for loop to find it and its index
// TODO: put in write lock here and read lock in derived_sensor_values()


void local_poses_cb(ConstLocalPosesStampedPtr &local_pose)
{
    sim_time = local_pose->time().sec() + (local_pose->time().nsec() * 10E-10);

    _sensor_pos(0) = local_pose->pose(0).position().x();
    _sensor_pos(1) = local_pose->pose(0).position().y();
    _sensor_pos(2) = local_pose->pose(0).position().z();

    _sensor_quat(0) = local_pose->pose(0).orientation().w();
    _sensor_quat(1) = local_pose->pose(0).orientation().x();
    _sensor_quat(2) = local_pose->pose(0).orientation().y();
    _sensor_quat(3) = local_pose->pose(0).orientation().z();


//    // for debugging quadcopter pose information
//    std::cout << local_pose->pose(0).name().data() << std::endl;
//    std::cout << local_pose->pose(0).position().x() << std::endl;
//    std::cout << local_pose->pose(0).position().y() << std::endl;
//    std::cout << local_pose->pose(0).position().z() << std::endl;
//
//    std::cout << local_pose->pose(0).orientation().w() << std::endl;
//    std::cout << local_pose->pose(0).orientation().x() << std::endl;
//    std::cout << local_pose->pose(0).orientation().y() << std::endl;
//    std::cout << local_pose->pose(0).orientation().z() << std::endl;
//    std::cout << std::endl;
}


int main(int _argc, char **_argv)
{
    // Load gazebo as a client
    gazebo::client::setup(_argc, _argv);

    // Create our node for communication
    node_handle = gazebo::transport::NodePtr(new gazebo::transport::Node());
    node_handle->Init();

    // Publish motor reference commands
    gazebo::transport::PublisherPtr pub0 =
        node_handle->Advertise<std_msgs::msgs::Float>("/gazebo/default/iris/ref/motor_speed/0", 1);
    gazebo::transport::PublisherPtr pub1 =
        node_handle->Advertise<std_msgs::msgs::Float>("/gazebo/default/iris/ref/motor_speed/1", 1);
    gazebo::transport::PublisherPtr pub2 =
        node_handle->Advertise<std_msgs::msgs::Float>("/gazebo/default/iris/ref/motor_speed/2", 1);
    gazebo::transport::PublisherPtr pub3 =
        node_handle->Advertise<std_msgs::msgs::Float>("/gazebo/default/iris/ref/motor_speed/3", 1);

    // TODO: implement a controller where the rotor velocities can be incorperated (I read this paper...)
//    // Subscribe to measured rotor velocities
//    gazebo::transport::SubscriberPtr sub0 = node_handle->Subscribe("/gazebo/default/iris/motor_speed/0", rotor0_cb);
//    gazebo::transport::SubscriberPtr sub1 = node_handle->Subscribe("/gazebo/default/iris/motor_speed/1", rotor1_cb);
//    gazebo::transport::SubscriberPtr sub2 = node_handle->Subscribe("/gazebo/default/iris/motor_speed/2", rotor2_cb);
//    gazebo::transport::SubscriberPtr sub3 = node_handle->Subscribe("/gazebo/default/iris/motor_speed/3", rotor3_cb);

    // Subscribe to measured position and orientation values
    gazebo::transport::SubscriberPtr sub5 = node_handle->Subscribe("~/pose/local/info", local_poses_cb);

    // TODO: organized this so the initial state comes from read-in values
    initialize_variables();         // assumes starting from origin with zeroed orientation

    Eigen::Matrix<double,1,4> desired_thrust_;

    std::ofstream testdata;
    testdata.open("test_data.txt");
    u_int64_t ctr = 0;
    while(1)
    {
        if(sim_state == 0)
        {
            prev_sim_time = sim_time;
            prev_sim_time_pos = sim_time;
            prev_sim_time_att = sim_time;
            sim_state = 2;
            std::cout << "Armed" << std::endl;
        } else if(sim_state == 2)
        {
            derived_sensor_values();                // called before a control decision
            if ((sim_time - prev_sim_time_pos) > 0.01){
                basic_position_controller();
                prev_sim_time_pos = sim_time;
            }
            if ((sim_time - prev_sim_time_att) > 0.001){
                basic_attitude_controller();
                prev_sim_time_att = sim_time;
            }

//            std::cout << desired_thrust_ << std::endl;
//            std::cout << std::endl;

            if (takeoff == _argv[1]) {
                if(((_desired_pos - _sensor_pos).lpNorm<2>() < 0.1) & _test == 1)
                {
//                    if((_desired_pos - _sensor_pos).lpNorm<2>() < 0.1)
                    _desired_pos << 1.0, 0.0, 2.0;
                    std::cout << "New desired position: " << _desired_pos << std::endl;
                    _test = 0;
                    _test1 = 1;
//                    _orig_desired_euler_att << 0.1, 0.0, 0.0;
//                    std::cout << "New desired attitude: " << _desired_euler_att << std::endl;
//                    _orig_desired_euler_att << 0.0, 0.0, 0.2;
//                    _orig_desired_euler_att << 0.0, 0.0, 0.2;
//                    _orig_desired_euler_att << 0.0, 0.0, 0.2;
//                    _orig_desired_euler_att << 0.0, 0.0, 0.2;
//                      _att_test = 1;
//                      _desired_pos << 0.5, 0.0, 2.0;
//                    if((_desired_euler_att - _derived_euler_att).lpNorm<2>() < 0.1)
                } else if (((_desired_pos - _sensor_pos).lpNorm<2>() < 0.1) & _test1 == 1)
                {
                    _desired_pos << 1.0, 1.0, 2.0;
                    std::cout << "New desired position: " << _desired_pos << std::endl;
                    _test1 = 0;
                    _test2 = 1;
                } else if (((_desired_pos - _sensor_pos).lpNorm<2>() < 0.1) & _test2 == 1)
                {
                    _desired_pos << 1.0, 1.0, 3.0;
                    std::cout << "New desired position: " << _desired_pos << std::endl;
                    _test2 = 0;
                    _test3 = 1;
                } else if (((_desired_pos - _sensor_pos).lpNorm<2>() < 0.1) & _test3 == 1)
                {
                    _desired_pos << 1.0, 1.0, 2.0;
                    std::cout << "New desired position: " << _desired_pos << std::endl;
                }
                test_cl_takeoff();

//                std::cout << "desired position: " << _desired_pos << std::endl;
//                std::cout << _sensor_pos << std::endl;
//                std::cout << "desired attitude: " << _desired_euler_att << std::endl;
//                std::cout << _derived_euler_att << std::endl;
//                std::cout << std::endl;
//                    test_ol_takeoff();
                if((ctr % 1000) == 0) {
                    testdata << sim_time << "\n";
                    testdata << std::fixed << std::setprecision(8) << _desired_pos << "\n";
                    testdata << std::fixed << std::setprecision(8) << _sensor_pos << "\n";
                    testdata << std::fixed << std::setprecision(8) << _desired_euler_att << "\n";
                    testdata << std::fixed << std::setprecision(8) << _derived_euler_att << "\n";
                    testdata << std::fixed << std::setprecision(8) << _desired_thrust << "\n";
                    testdata << std::fixed << std::setprecision(8) << _final_att_deltas << "\n";

                }

                ctr++;          // TODO: need to handle this overflow, maybe use some sort of circular buffer
            } else {
                test_ol_land();
            }

            pub0->Publish(ref_motor_vel0);
            pub1->Publish(ref_motor_vel1);
            pub2->Publish(ref_motor_vel2);
            pub3->Publish(ref_motor_vel3);
        }
    } // end while(1)

} // end main()

void test_ol_takeoff()
{
    ref_motor_vel0.set_data(690.0);
    ref_motor_vel1.set_data(690.0);
    ref_motor_vel2.set_data(690.0);     // TODO: not sure why there's a delta now, might be that the props are perfectly centered now
    ref_motor_vel3.set_data(690.0);
}

void test_ol_land()
{
    // 665.0 is the hover point
    ref_motor_vel0.set_data(645.0);
    ref_motor_vel1.set_data(645.0);
    ref_motor_vel2.set_data(645.0);
    ref_motor_vel3.set_data(645.0);
}

void test_cl_takeoff()
{
//    _desired_thrust = _desired_thrust / 10.0;
    ref_motor_vel0.set_data(_desired_thrust(0));
    ref_motor_vel1.set_data(_desired_thrust(1));
    ref_motor_vel2.set_data(_desired_thrust(2));
    ref_motor_vel3.set_data(_desired_thrust(3));
}

void initialize_variables()
{
////     Soft controller
//    _Kp_pos << 3.0, 3.0, 10.0;
//    _Kd_pos << 3.0, 3.0, 6.0;
//    _Kp_ang << 700.0, 700.0, 0.0;
//    _Kd_ang << 100000.0, 100000.0, 0.0;

//    //  Testing/Intermediate Controller
//    _Kp_pos << 5.0, 5.0, 10.0;
//    _Kd_pos << 3.0, 3.0, 6.0;
//    _Kp_ang << 700.0, 700.0, 0.0;
//    _Kd_ang << 100000.0, 100000.0, 0.0;

    // Fast/Stiff controller
    _Kp_pos << 8.0, 8.0, 32.0;
    _Kd_pos << 3.8, 4.1, 10.0;
    _Kp_ang << 1100.0, 1100.0, 0.0;
    _Kd_ang << 120000.0, 120000.0, 0.0;

    _desired_pos << 0.0, 0.0, 2.0;
    _desired_vel << 0.0, 0.0, 0.0;
    _desired_acc << 0.0, 0.0, 0.0;
    _desired_euler_att << 0.0, 0.0, 0.0; // roll, pitch, yaw
    _orig_desired_euler_att = _desired_euler_att;
//    _desired_euler_att(2) = quat2euler(_sensor_quat)(2);
    _desired_pqr_att << 0.0, 0.0, 0.0;          // angular rates (not really the "velocity")

    _state << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;   // zeroed position and orientation
    _statedot << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    _sensor_quat << 1.0, 0.0, 0.0, 0.0;                                     // define upright zeroed orientation
    _quat_normalized << 1.0, 0.0, 0.0, 0.0;
    _derived_quatdot << 0.0, 0.0, 0.0, 0.0;
    _z_bbasis << 0.0, 0.0, 1.0;
    _blin_force << 0.0, 0.0, 0.0;
    _derived_euler_att << 0.0, 0.0, 0.0;
    _final_att_deltas << 0.0, 0.0, 0.0;

} // end initialize_variables()

void derived_sensor_values()
{
//    lin_vel_x lin_vel_y lin_vel_z ang_vel_p ang_vel_q ang_vel_r
    if ((sim_state == 2) & ((sim_time - prev_sim_time) > 0.000001))                      // currently running at the micro-second scale
    {
        // TODO: sometimes I get a 0.0 evaluation for lin_vel_z between otherwise good values, not sure why
        sim_time_delta = sim_time - prev_sim_time;
        _derived_lin_vel(0) = (_sensor_pos(0) - _prev_sensor_pos(0)) / sim_time_delta;
        _derived_lin_vel(1) = (_sensor_pos(1) - _prev_sensor_pos(1)) / sim_time_delta;
        _derived_lin_vel(2) = (_sensor_pos(2) - _prev_sensor_pos(2)) / sim_time_delta;

        Eigen::Matrix<double,1,3> euler_ = quat2euler(_sensor_quat);
//        euler_ = _imu_alignment * (euler_.transpose());
//        _derived_euler_attdot = euler_ - _prev_derived_euler_att; // TODO: check this for accuracy
        _derived_pqr_att = derive_ang_velocity(euler_);

        _prev_sensor_pos(0) = _sensor_pos(0);
        _prev_sensor_pos(1) = _sensor_pos(1);
        _prev_sensor_pos(2) = _sensor_pos(2);

        _prev_derived_euler_att(0) = euler_(0);
        _prev_derived_euler_att(1) = euler_(1);
//        _prev_derived_euler_att(1) = -euler_(1);
        _prev_derived_euler_att(2) = euler_(2);


        prev_sim_time = sim_time;                   // should be the last thing run
    }

} // end derived_sensor_values()

//references (papers):
// Trajectory Generation and Control for Precise Aggressive Maneuvers with Quadrotors
// The GRASP Multiple Micro UAV Testbed
// TODO: is the position in body frame converted from world frame??
void basic_position_controller()
{
    // TODO: i'm pretty sure we shouldn't be keeping track of _desired_acc (eg: every loop it iterates)

    Eigen::Array3d acc_des_ = _desired_acc + (1.0*_Kd_pos.cwiseProduct(_desired_vel - _derived_lin_vel))
                                + (1.0*_Kp_pos.cwiseProduct(_desired_pos - _sensor_pos));

//    Eigen::Matrix<double,1,3> _desired_acc;
//    _desired_acc(0) = _gravity * ((_desired_euler_att(1) * cos(_desired_euler_att(2)))
//                                + (_desired_euler_att(0) * sin(_desired_euler_att(2))));
//    _desired_acc(1) = _gravity * ((_desired_euler_att(1) * sin(_desired_euler_att(2)))
//                                  - (_desired_euler_att(0) * cos(_desired_euler_att(2))));

    //TODO: need to constrain the attitude quaternion to a norm of 1...
//    _desired_euler_att(0) = (1.0/_gravity) * ((acc_des_(0)*sin(_desired_euler_att(2))) - (acc_des_(1)*cos(_desired_euler_att(2))));
//    _desired_euler_att(1) = (1.0/_gravity) * ((acc_des_(0)*cos(_desired_euler_att(2))) + (acc_des_(1)*sin(_desired_euler_att(2))));
    _desired_euler_att(0) = (1.0/_gravity) * ((acc_des_(0)*sin(_desired_euler_att(2))) + (acc_des_(1)*cos(_desired_euler_att(2))));
    _desired_euler_att(1) = (1.0/_gravity) * ((acc_des_(0)*-1.0*cos(_desired_euler_att(2))) + (acc_des_(1)*sin(_desired_euler_att(2))));
//    _desired_euler_att(2) = _derived_euler_att(2);        // desired yaw is forward-facing
    _desired_euler_att(2) = _orig_desired_euler_att(2);

    // TODO: there is a problem here!! either _desired_euler_att shouldn't be updated to itself (above)
    // TODO: we might not even be needing to update the desired_euler_att here...
    // TODO: THERE IS A MISALIGNMENT BETWEEN "FORWARD" FOR THE QUAD AND "FORWARD" FOR THE EULER ANGLE MEASUREMENT
//    _desired_euler_att(0) = _orig_desired_euler_att(0);
//    _desired_euler_att(1) = _orig_desired_euler_att(1);
//    _desired_euler_att(2) = _orig_desired_euler_att(2);


    // TODO: the desired total thrust for 1) seems kinda low, 2) better...
    if(_att_test){
        _desired_tot_thrust_delta = _hover_point;
    } else{
        _desired_tot_thrust_delta = (_mass / (8.0 * _motor_force_const * _hover_point)) * acc_des_(2);
    }

//    std::cout << _desired_euler_att << std::endl;
//    std::cout << _desired_tot_thrust_delta << std::endl;
//    std::cout << std::endl;

} // end basic_position_controller()

Eigen::Matrix<double,1,3> test_att_;

//TODO: need to fix the desired attitude so I can test the controller
//TODO: right now it's updating from the position controller
void basic_attitude_controller()
{
//    /// Test
//    _bRw = quat2rot(_sensor_quat);
//    _wRb = _bRw.transpose();
//
//    test_att_ = _wRb*_derived_euler_att;
//    ///

    _derived_euler_att = quat2euler(_sensor_quat);
//    _derived_euler_att(1) = -_derived_euler_att(1);
//    _derived_euler_att = _imu_alignment * (_derived_euler_att.transpose());

    Eigen::Array3d att_deltas_;

    // TODO: the roll is getting away from me, i don't know why
    // TODO: don't forget that these signs have be flip flopped around, I think negative is correct...
    att_deltas_(0) = 1.0*(_Kp_ang(0) * (_desired_euler_att(0) - _derived_euler_att(0)))
                            + 1.0*(_Kd_ang(0) * (_desired_pqr_att(0) - _derived_pqr_att(0)));
    att_deltas_(1) = 1.0*(_Kp_ang(1) * (_desired_euler_att(1) - _derived_euler_att(1)))
                            + 1.0*(_Kd_ang(1) * (_desired_pqr_att(1) - _derived_pqr_att(1)));
    att_deltas_(2) = 1.0*(_Kp_ang(2) * (_desired_euler_att(2) - _derived_euler_att(2)))
                            + 1.0*(_Kd_ang(2) * (_desired_pqr_att(2) - _derived_pqr_att(2)));

    _final_att_deltas = att_deltas_;
//    att_deltas_ = -1.0 * att_deltas_;
//    att_deltas_(0) = -1.0 * att_deltas_(0);
//    att_deltas_(1) = -1.0 * att_deltas_(1);
//    att_deltas_(2) = 0.0;

    Eigen::Matrix<double,4,1> all_deltas_;
    all_deltas_ << (_hover_point + _desired_tot_thrust_delta), att_deltas_(0), att_deltas_(1), att_deltas_(2);

    _desired_thrust =  (_motor_mapping * all_deltas_);


    // TODO: is this an issue with coordinate frames? what frame is my quaternion in?
    // TODO: if the quaternion measurement is based in the inertial frame it's gonna f everything up
    // TODO: this also explains why the hovering is sort of okay
    // TODO: I'M NOT DOING ANY ROTATIONAL TRANSFORMS, THIS IS A PROBLEM
} // end basic_attitude_controller()

// helper functions
// https://stackoverflow.com/questions/28585653/use-of-lpnorm-in-eigen
// https://eigen.tuxfamily.org/dox/TopicFunctionTakingEigenTypes.html
// https://stackoverflow.com/questions/35612831/eigenref-in-pass-by-pointer
Eigen::Matrix3d quat2rot(const Eigen::Ref<const Eigen::Matrix<double,1,4>>& q_)
{
    Eigen::Matrix3d rotation_;

    _quat_normalized = q_.normalized();
    _q_hat(0,0) = 0.0;                            _q_hat(0,1) = -1.0*_quat_normalized(3); _q_hat(0,2) = _quat_normalized(2);
    _q_hat(1,0) = _quat_normalized(3);      _q_hat(1,1) = 0.0;                            _q_hat(1,2) = -1.0*_quat_normalized(1);
    _q_hat(2,0) = -1.0*_quat_normalized(2); _q_hat(2,1) = _quat_normalized(1);      _q_hat(2,2) = 0.0;

    rotation_ = Eigen::Matrix3d::Identity() + (2.0 * _q_hat * _q_hat) + (2.0 * _quat_normalized(0) * _q_hat);

    return(rotation_);
} // end quat2rot()

// TODO: could approximate atan2...
// TODO: https://www.dsprelated.com/showarticle/1052.php
Eigen::Matrix<double,1,3> quat2euler(const Eigen::Ref<const Eigen::Matrix<double,1,4>>& q_)
{
    Eigen::Matrix3d rotation_;
    Eigen::Matrix<double,1,3> euler_;
    euler_ << 0.0, 0.0, 0.0;

    rotation_ = quat2rot(q_);
    euler_(0) = asin(rotation_(1,2));   // roll
    euler_(1) = atan2((-1.0*rotation_(0,2)) / cos(euler_(0)),
                                   (rotation_(2,2) / cos(euler_(0))));
    euler_(2) = atan2((-1.0*rotation_(1,0)) / cos(euler_(0)),
                      (rotation_(1,1) / cos(euler_(0))));
//    std::cout << euler_ << std::endl;

    return(euler_);
} // end quat2euler()

Eigen::Matrix<double,1,3> derive_ang_velocity(const Eigen::Ref<const Eigen::Matrix<double,1,3>>& e_)
{
    Eigen::Matrix<double,3,3> tfm_;
    tfm_(0,0) = cos(e_(1)); tfm_(0,1) = 0.0; tfm_(0,2) = (-1.0*cos(e_(0))*sin(e_(1)));
    tfm_(1,0) = 0.0; tfm_(1,1) = 1.0; tfm_(1,2) = sin(e_(0));
    tfm_(2,0) = sin(e_(1)); tfm_(2,1) = 0.0; tfm_(2,2) = (cos(e_(0))*cos(e_(1)));

    _derived_pqr_att = tfm_ * (e_ - _prev_derived_euler_att).transpose();

    return(_derived_pqr_att);
}
#pragma clang diagnostic pop