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
    gazebo::transport::SubscriberPtr sub5 = node_handle->Subscribe("~/pose/info", local_poses_cb);

    // TODO: organized this so the initial state comes from read-in values
    initialize_variables();         // assumes starting from origin with zeroed orientation

    Eigen::Matrix<double,1,4> desired_thrust_;

    while(1)
    {
        if(sim_state == 0)
        {
            prev_sim_time = sim_time;
            sim_state = 2;
            std::cout << "Armed" << std::endl;
        } else if(sim_state == 2)
        {
            derived_sensor_values();                // called before a control decision
            basic_position_controller();
            basic_attitude_controller();

//            std::cout << desired_thrust_ << std::endl;
//            std::cout << std::endl;

            if (takeoff == _argv[1]) {
                test_cl_takeoff();
            } else {
                test_ol_land();
            }

            // publish motor commands
//            pub0->Publish(ref_motor_vel0);
//            pub1->Publish(ref_motor_vel1);
//            pub2->Publish(ref_motor_vel2);
//            pub3->Publish(ref_motor_vel3);

            std::cout << _derived_euler_att << std::endl;
            std::cout << _desired_euler_att << std::endl;
            std::cout << _desired_thrust << std::endl;
            std::cout << std::endl;

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
    ref_motor_vel2.set_data(690.0);
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
    ref_motor_vel0.set_data(_desired_thrust(0));
    ref_motor_vel1.set_data(_desired_thrust(1));
    ref_motor_vel2.set_data(_desired_thrust(2));
    ref_motor_vel3.set_data(_desired_thrust(3));
}

void initialize_variables()
{
//    _Kp_pos << 15.0, 15.0, 30.0;
//    _Kd_pos << 12.0, 12.0, 10.0;
//    _Kp_ang << 3000.0, 3000.0, 3000.0;
//    _Kd_ang << 300.0, 300.0, 300.0;
    _Kp_pos << 1.0, 1.0, 8.0;
    _Kd_pos << 0.0, 0.0, 4.0;
    _Kp_ang << 0.0, 0.0, 50.0;
    _Kd_ang << 0.0, 00.0, 00.0;

    _desired_pos << 0.1, 0.0, 2.0;
    _desired_vel << 0.0, 0.0, 0.0;
    _desired_acc << 0.0, 0.0, 0.0;
    _desired_euler_att << 0.0, 0.0, 0.0;
    _desired_pqr_att << 0.0, 0.0, 0.0;          // angular rates (not really the "velocity")

    _state << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;   // zeroed position and orientation
    _statedot << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    _sensor_quat << 1.0, 0.0, 0.0, 0.0;                                     // define upright zeroed orientation
    _quat_normalized << 1.0, 0.0, 0.0, 0.0;
    _derived_quatdot << 0.0, 0.0, 0.0, 0.0;
    _z_bbasis << 0.0, 0.0, 1.0;
    _blin_force << 0.0, 0.0, 0.0;
    _derived_euler_att << 0.0, 0.0, 0.0;

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
//        _derived_euler_attdot = euler_ - _prev_derived_euler_att; // TODO: check this for accuracy
        _derived_pqr_att = derive_ang_velocity(euler_);

        _prev_sensor_pos(0) = _sensor_pos(0);
        _prev_sensor_pos(1) = _sensor_pos(1);
        _prev_sensor_pos(2) = _sensor_pos(2);

        _prev_derived_euler_att(0) = euler_(0);
        _prev_derived_euler_att(1) = euler_(1);
        _prev_derived_euler_att(2) = euler_(2);


        prev_sim_time = sim_time;                   // should be the last thing run
    }

} // end derived_sensor_values()

//references:
// Trajectory Generation and Control for Precise Aggressive Maneuvers with Quadrotors
// The GRASP Multiple Micro UAV Testbed
void basic_position_controller()
{
    // TODO: i'm pretty sure we shouldn't be keeping track of _desired_acc (eg: every loop it iterates)

    Eigen::Array3d acc_des_ = _desired_acc + (_Kd_pos * (_desired_vel - _derived_lin_vel))
                                + (_Kp_pos * (_desired_pos - _sensor_pos));

    _desired_euler_att(0) = (1.0/_gravity) * ((acc_des_(0)*sin(0.0)) - (acc_des_(1)*cos(0.0)));
    _desired_euler_att(1) = (1.0/_gravity) * ((acc_des_(0)*cos(0.0)) + (acc_des_(1)*sin(0.0)));
    _desired_euler_att(2) = 0.0;        // desired yaw is 0.0, forward-facing

    // TODO: the desired total thrust for 1) seems kinda low, 2) better...
//    _desired_tot_thrust = _mass * (_gravity + acc_des_(2));
    _desired_tot_thrust_delta = (_mass / (8.0 * _motor_force_const * _hover_point)) * acc_des_(2);

//    std::cout << _desired_euler_att << std::endl;
//    std::cout << _desired_tot_thrust_delta << std::endl;
//    std::cout << std::endl;

} // end basic_position_controller()

void basic_attitude_controller()
{
    _derived_euler_att = quat2euler(_sensor_quat);

    Eigen::Array3d att_deltas_;
    att_deltas_(0) = (_Kp_ang(0) * (_desired_euler_att(0) - _derived_euler_att(0)))
                            + (_Kd_ang(0) * (_desired_pqr_att(0) - _derived_pqr_att(0)));
    att_deltas_(1) = (_Kp_ang(1) * (_desired_euler_att(1) - _derived_euler_att(1)))
                            + (_Kd_ang(1) * (_desired_pqr_att(1) - _derived_pqr_att(1)));
    att_deltas_(2) = (_Kp_ang(2) * (_desired_euler_att(2) - _derived_euler_att(2)))
                            + (_Kd_ang(2) * (_desired_pqr_att(2) - _derived_pqr_att(2)));

//    att_deltas_ = -1.0 * att_deltas_;
//    att_deltas_(0) = -1.0 * att_deltas_(0);
//    att_deltas_(1) = -1.0 * att_deltas_(1);
//    att_deltas_(2) = 0.0;

    Eigen::Matrix<double,4,1> all_deltas_;
    all_deltas_ << (_hover_point + _desired_tot_thrust_delta), att_deltas_(0), att_deltas_(1), att_deltas_(2);

    _desired_thrust =  (_motor_mapping * all_deltas_);
    std::cout << std::endl;
    std::cout << "mappings" << std::endl;
    std::cout << _motor_mapping << std::endl;
    std::cout << att_deltas_ << std::endl;
    std::cout << std::endl;
    std::cout << all_deltas_ << std::endl;
    std::cout << _desired_thrust << std::endl;
    std::cout << std::endl;

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