//
// Created by odysseus on 9/5/20.
//

#ifndef SET_ROTOR_VEL_HPP
#define SET_ROTOR_VEL_HPP

#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/common.hh>
#include <string.h>
#include <cmath>
#include <eigen3/Eigen/Eigen>          // we have options here: ignition (limited), xtensor (like numpy), eigen (standard?)
#include <iostream>
#include <fstream>
//#include <eigen3/Eigen/Core>

#include <gazebo/gazebo_client.hh>
#include "../msgs/include/Float.pb.h"
#include "../msgs/include/Wind.pb.h"
#include "../msgs/include/CommandMotorSpeed.pb.h"
#include "../msgs/include/local_poses_stamped.pb.h"

// gazebo related variables
gazebo::transport::NodePtr node_handle;
typedef const boost::shared_ptr<const std_msgs::msgs::Float> MotorSpeedPtr;         // points to protobuf custom message "Float"
typedef const boost::shared_ptr<const gazebo::msgs::LocalPosesStamped> ConstLocalPosesStampedPtr;

// simulation time
double sim_time = 0.0;
double prev_sim_time = 0.0;
double prev_sim_time_pos = 0.0;
double prev_sim_time_att = 0.0;
double sim_time_delta = 0.0;

// simulation state
int sim_state = 0;              // 0 - not armed, 1 - pre-armed, 2 - armed

// Reference motor values that are published
std_msgs::msgs::Float ref_motor_vel0;
std_msgs::msgs::Float ref_motor_vel1;
std_msgs::msgs::Float ref_motor_vel2;
std_msgs::msgs::Float ref_motor_vel3;

// Measured motor values
double sensor_motor_vel0 = 0.0;
double sensor_motor_vel1 = 0.0;
double sensor_motor_vel2 = 0.0;
double sensor_motor_vel3 = 0.0;

// Previous position and orientation values
double prev_sensor_rot_x = 0.0;
double prev_sensor_rot_y = 0.0;
double prev_sensor_rot_z = 0.0;
double prev_sensor_rot_w = 0.0;

// http://ais.informatik.uni-freiburg.de/publications/papers/sittel13tr.pdf
// Previous position and orientation values (leave enough time for calculations to complete)
// derived linear and angular velocity
double ang_vel_p = 0.0;
double ang_vel_q = 0.0;
double ang_vel_r = 0.0;

// Actions
std::string takeoff ("takeoff");


// TODO: I could probably pull this directly from the SDF file, not via head, but maybe dynamically at program start
// TODO: use Eigen/Sparse and it's functions to reduce memory use, most of my matrices are sparse

// environmental variables
double _gravity = 9.81;     // m/s^2

bool _test = 1;
bool _test1 = 0;
bool _test2 = 0;
bool _test3 = 0;

// initialize physical properties of the vehicle
double _mass = 1.5;          // kg
double _length = 0.2555;     // m
double _hover_point = 665.0; // rad/s for one motor only
double _motor_force_const = 8.54858e-06;
double _motor_torque_const = 0.016;         // TODO: seems too large
//Eigen::Matrix4d _motor_mapping((Eigen::Matrix4d() << 1.0, 0.0, -1.0, 1.0, 1.0, 1.0, 0.0, -1.0, 1.0, 0.0, 1.0, 1.0, 1.0, -1.0, 0.0, -1.0).finished());
Eigen::Matrix4d _motor_mapping((Eigen::Matrix4d() << 1.0, 0.5, 0.5, 1.0, 1.0, -0.5, 0.5, -1.0, 1.0, -0.5, -0.5, 1.0, 1.0, 0.5, -0.5, -1.0).finished());


static Eigen::Matrix3d _I((Eigen::Matrix3d() << 0.029125, 0.0, 0.0, 0.0, 0.029125, 0.0, 0.0, 0.0, 0.029125).finished());
static Eigen::Matrix3d _I_inv = _I.inverse();
Eigen::Matrix3d _bRw((Eigen::Matrix3d() << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished());
Eigen::Matrix3d _wRb((Eigen::Matrix3d() << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished());
//Eigen::Matrix3d _imu_alignment((Eigen::Matrix3d() << cos(-0.785398), -1.0*sin(-0.785398), 0.0, -1.0*sin(-0.785398), cos(-0.785398), 0.0, 0.0, 0.0, 1.0).finished());
//Eigen::Matrix4d _quat_matrix((Eigen::Matrix4d() << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished());
Eigen::Matrix3d _q_hat((Eigen::Matrix3d() << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished());
Eigen::Matrix<double,1,4> _sensor_quat((Eigen::Matrix<double,1,4>() << 1.0, 0.0, 0.0, 0.0).finished());
Eigen::Matrix<double,1,4> _quat_normalized((Eigen::Matrix<double,1,4>() << 1.0, 0.0, 0.0, 0.0).finished());
Eigen::Matrix<double,1,4> _desired_thrust((Eigen::Matrix<double,1,4>() << 1.0, 0.0, 0.0, 0.0).finished());
Eigen::Array<double,1,13> _state;
Eigen::Array<double,1,13> _statedot;
Eigen::Matrix<double,1,3> _sensor_pos;
Eigen::Matrix<double,1,3> _prev_sensor_pos;
Eigen::Matrix<double,1,3> _derived_lin_vel;
Eigen::Matrix<double,1,3> _derived_euler_att;
Eigen::Matrix<double,1,3> _derived_euler_attdot;
Eigen::Matrix<double,1,3> _prev_derived_euler_att;
Eigen::Matrix<double,1,3> _derived_pqr_att;
Eigen::Array<double,1,4> _derived_quatdot;
Eigen::Array<double,1,3> _z_bbasis;      // z basis vector for body frame
Eigen::Array<double,1,3> _blin_force;    // linear force for body frame
Eigen::Matrix<double,1,3> _final_att_deltas;

Eigen::Matrix<double,1,3> _desired_pos;
Eigen::Matrix<double,1,3> _desired_vel;
Eigen::Matrix<double,1,3> _desired_acc;
Eigen::Matrix<double,1,3> _desired_euler_att;
Eigen::Matrix<double,1,3> _orig_desired_euler_att;
Eigen::Array<double,1,3> _desired_pqr_att;
double _desired_tot_thrust_delta;

static Eigen::Matrix<double,1,3> _Kp_pos;
static Eigen::Matrix<double,1,3> _Kd_pos;
static Eigen::Matrix<double,1,3> _Kp_ang;
static Eigen::Matrix<double,1,3> _Kd_ang;

// test
bool _att_test = 0;

// function declarations
void test_ol_takeoff();
void test_ol_land();
void test_cl_takeoff();
void derived_sensor_values();
void initialize_variables();
void basic_position_controller();
void basic_attitude_controller();


void rotor0_cb(MotorSpeedPtr &rotor_vel);
void rotor1_cb(MotorSpeedPtr &rotor_vel);
void rotor2_cb(MotorSpeedPtr &rotor_vel);
void rotor3_cb(MotorSpeedPtr &rotor_vel);

Eigen::Matrix3d quat2rot(const Eigen::Ref<const Eigen::Matrix<double,1,4>>& q);
Eigen::Matrix<double,1,3> quat2euler(const Eigen::Ref<const Eigen::Matrix<double,1,4>>& q);
Eigen::Matrix<double,1,3> derive_ang_velocity(const Eigen::Ref<const Eigen::Matrix<double,1,3>>& e_);

#endif // SET_ROTOR_VEL_HPP
