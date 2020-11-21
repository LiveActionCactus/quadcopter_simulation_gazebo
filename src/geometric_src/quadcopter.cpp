// Quadcopter member function definitions
//
// By: Patrick Ledzian
// Date: 11 November 2020
//

#include "../../include/geometric_include/quadcopter.hpp"

//// Quadcopter class initializations and definitions
Quadcopter::Quadcopter()
    :   trajectory(Trajectory()),
        sim_time_(0.0),                      // seconds
        prev_sim_time_(0.0),
        sim_time_delta_(0.0),
        sim_state_(0),
        ref_motor_vel0_(),                   // default constructed Float protobuf messages
        ref_motor_vel1_(),
        ref_motor_vel2_(),
        ref_motor_vel3_(),
        gravity_(9.81),                     // m/s^2
        mass_(1.5),                         // kg
        hover_point_(665.0),                // rad/s for one motor only
        motor_force_const_(8.54858e-06),
        motor_torque_const_(0.016),
        arm_length_(0.1784),                // m
        motor_force_mapping_(),             // default constructed Eigen matrices (all zeros)
        sensor_pos_(),                      // m
        prev_sensor_pos_(),
        derived_lin_vel_(),                 // m/s
        sensor_quat_(),
        derived_euler_att_(),               // rad
        prev_derived_euler_att_(),
        derived_pqr_att_(),                 // rad/s
//        desired_pos_(),                     // m
//        desired_vel_(),                     // m/s
//        desired_acc_(),                     // m/s^{2}
//        desired_euler_att_(),               // rad
//        desired_pqr_att_(),                 // rad/s
//        desired_thrust_(),                  // rad/s
        final_att_deltas_()
{
    // Initialize Gazebo variables
    node_handle_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    node_handle_->Init();
    sub = node_handle_->Subscribe("~/pose/local/info", &Quadcopter::local_poses_cb, this);         //
    pub0 = node_handle_->Advertise<std_msgs::msgs::Float>("/gazebo/default/iris/ref/motor_speed/0", 1);
    pub1 = node_handle_->Advertise<std_msgs::msgs::Float>("/gazebo/default/iris/ref/motor_speed/1", 1);
    pub2 = node_handle_->Advertise<std_msgs::msgs::Float>("/gazebo/default/iris/ref/motor_speed/2", 1);
    pub3 = node_handle_->Advertise<std_msgs::msgs::Float>("/gazebo/default/iris/ref/motor_speed/3", 1);

    // Initialize remaining variables
    motor_force_mapping_ << 1.0,                      0.5,                      0.5,                      1.0,
                            0.5*arm_length_,         -0.5*arm_length_,         -0.5*arm_length_,          0.5*arm_length_,
                            0.5*arm_length_,          0.5*arm_length_,         -0.5*arm_length_,         -0.5*arm_length_,
                            1.0*motor_torque_const_,  0.5*motor_torque_const_, -0.5*motor_torque_const_, -1.0*motor_torque_const_;
    inv_motor_force_mapping_ = motor_force_mapping_.inverse();
    J_ << 0.029125,  0.0,       0.0,
          0.0,       0.029125,  0.0,
          0.0,       0.0,       0.055225;
    sensor_quat_ << 1.0, -1.0, 1.0, -1.0;
} // end Quadcopter::Quadcopter()

Quadcopter::~Quadcopter()
{
    sub->Unsubscribe();         // clean-up the spawned subscriber threads
}

//// Gazebo subscriber callback function -- stores quadcopter states
void Quadcopter::local_poses_cb(ConstLocalPosesStampedPtr &local_pose)
{
    sim_time_ = local_pose->time().sec() + (local_pose->time().nsec() * 10E-10);

    sensor_pos_(0) = local_pose->pose(0).position().x();
    sensor_pos_(1) = local_pose->pose(0).position().y();
    sensor_pos_(2) = local_pose->pose(0).position().z();

    sensor_quat_(0) = local_pose->pose(0).orientation().w();
    sensor_quat_(1) = local_pose->pose(0).orientation().x();
    sensor_quat_(2) = local_pose->pose(0).orientation().y();
    sensor_quat_(3) = local_pose->pose(0).orientation().z();
} // end Quadcopter::local_poses_cb()

//// run parent loop of the vehicle
void Quadcopter::run()
{
    update_state_and_data();
    if(sim_time_ > 5.0){
        trajectory.set_new_trajectory("hover");
    }
    trajectory.run_trajectory_update();
    controller.position_control(*this, trajectory);
    controller.attitude_control(*this, trajectory);
    controller.set_rotor_rates(*this);
    publish_rotor_cmds();
}

//// update data and state machine
void Quadcopter::update_state_and_data()
{
    // TODO: implement check for state updates....

    // update data
    switch(sim_state_) {
        case 0:
            if (sim_time_ > 3.0) {
                sim_state_ = 1;
            }
            break;
        case 1:
            sim_state_ = 2;
            break;
        case 2:
            if ((sim_time_ - prev_sim_time_) > 0.000001)
            {
                derived_sensor_values();
            }
            break;
    } // end switch(sim_state_)

} // end Quadcopter::update_state_and_data()

//// Derive sensor values from position and orientation measurements
void Quadcopter::derived_sensor_values()
{
    // TODO: sometimes I get a 0.0 evaluation for lin_vel_z between otherwise good values, not sure why
    // TODO: some of these pitch values are an order of magnitude or more off
    sim_time_delta_ = sim_time_ - prev_sim_time_;              // time slice used to derive velocity values

    derived_lin_vel_(0) = (sensor_pos_(0) - prev_sensor_pos_(0)) / sim_time_delta_;
    derived_lin_vel_(1) = (sensor_pos_(1) - prev_sensor_pos_(1)) / sim_time_delta_;
    derived_lin_vel_(2) = (sensor_pos_(2) - prev_sensor_pos_(2)) / sim_time_delta_;

    derived_rot_ = quat2rot(sensor_quat_);
    derived_euler_att_ = quat2euler(sensor_quat_);                      // convert from quaternion to euler angles

    derived_pqr_att_ = derive_ang_velocity(derived_euler_att_, prev_derived_euler_att_);         // produces angular velocity vector

    // store previous position value
    prev_sensor_pos_(0) = sensor_pos_(0);
    prev_sensor_pos_(1) = sensor_pos_(1);
    prev_sensor_pos_(2) = sensor_pos_(2);

    // store previous attitude value
    prev_derived_euler_att_(0) = derived_euler_att_(0);
    prev_derived_euler_att_(1) = derived_euler_att_(1);
    prev_derived_euler_att_(2) = derived_euler_att_(2);

    prev_sim_time_ = sim_time_;                   // should be the last thing run
} // end Quadcopter::derived_sensor_values()

//
// Helper member functions
//

//// Publish commanded rotor velocities (rad/s)
void Quadcopter::publish_rotor_cmds()
{
//    std::cout << "rotor rates: " << std::endl;
//    std::cout << ref_motor_vel0_.data() << std::endl;
//    std::cout << ref_motor_vel1_.data() << std::endl;
//    std::cout << ref_motor_vel2_.data() << std::endl;
//    std::cout << ref_motor_vel3_.data() << std::endl;
//    std::cout << std::endl;

    pub0->Publish(ref_motor_vel0_);
    pub1->Publish(ref_motor_vel1_);
    pub2->Publish(ref_motor_vel2_);
    pub3->Publish(ref_motor_vel3_);

//    std_msgs::msgs::Float test;
//    test.set_data(100.0);
//    pub0->Publish(test);
//    pub1->Publish(test);
//    pub2->Publish(test);
//    pub3->Publish(test);
}

//// Convert quaternions to euler angles
Eigen::Matrix<double,1,3> Quadcopter::quat2euler(const Eigen::Ref<const Eigen::Matrix<double,1,4>>& q_)
{
    // References
    // previously listed papers
    // Nice approximation of atan2: https://www.dsprelated.com/showarticle/1052.php
    // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    // http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/
    //

    Eigen::Matrix<double,3,3> rotation_;
    Eigen::Matrix<double,1,3> euler_;
    euler_ << 0.0, 0.0, 0.0;

    rotation_ = quat2rot(q_);
    euler_(0) = asin(rotation_(1,2));   // roll
    euler_(1) = atan2((-1.0*rotation_(0,2)) / cos(euler_(0)),
                      (rotation_(2,2) / cos(euler_(0))));
    euler_(2) = atan2((-1.0*rotation_(1,0)) / cos(euler_(0)),
                      (rotation_(1,1) / cos(euler_(0))));

    return(euler_);

} // end Quadcopter::quat2euler()

//// Convert quaternions to a rotation matrix -- used in quat2euler()
Eigen::Matrix<double,3,3> Quadcopter::quat2rot(const Eigen::Ref<const Eigen::Matrix<double,1,4>>& q_)
{
    Eigen::Matrix<double,3,3> q_hat_;
    Eigen::Matrix<double,1,4> quat_normalized_;

    quat_normalized_ = q_.normalized();
    q_hat_(0,0) = 0.0;                              q_hat_(0,1) = -1.0*quat_normalized_(3);    q_hat_(0,2) = quat_normalized_(2);
    q_hat_(1,0) = quat_normalized_(3);        q_hat_(1,1) = 0.0;                               q_hat_(1,2) = -1.0*quat_normalized_(1);
    q_hat_(2,0) = -1.0*quat_normalized_(2);   q_hat_(2,1) = quat_normalized_(1);         q_hat_(2,2) = 0.0;

    return(Eigen::Matrix<double,3,3>::Identity() + (2.0 * q_hat_ * q_hat_) + (2.0 * quat_normalized_(0) * q_hat_));   // returns a rotation matrix

} // end Quadcopter::quat2rot()

//// Derives angular velocity vector from euler angles
Eigen::Matrix<double,1,3> Quadcopter::derive_ang_velocity(const Eigen::Ref<const Eigen::Matrix<double,1,3>>& e_, const Eigen::Ref<const Eigen::Matrix<double,1,3>>& prev_e_)
{
    // References:
    // The GRASP Multiple Micro UAV Testbed
    //

    Eigen::Matrix<double,3,3> tfm_;             // transformation matrix

    tfm_(0,0) = cos(e_(1)); tfm_(0,1) = 0.0; tfm_(0,2) = (-1.0*cos(e_(0))*sin(e_(1)));
    tfm_(1,0) = 0.0;                 tfm_(1,1) = 1.0; tfm_(1,2) = sin(e_(0));
    tfm_(2,0) = sin(e_(1)); tfm_(2,1) = 0.0; tfm_(2,2) = (cos(e_(0))*cos(e_(1)));

    return(tfm_ * (e_ - prev_e_).transpose());       // angular velocity vector

} // end Quadcopter::derive_ang_velocity()