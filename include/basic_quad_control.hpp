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
#include <eigen3/Eigen/Eigen>          // we have options here: ignition (limited), xtensor (like numpy), eigen (standard?)

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

// Measured position and orientation values
double sensor_pos_x = 0.0;
double sensor_pos_y = 0.0;
double sensor_pos_z = 0.0;
double sensor_rot_x = 0.0;
double sensor_rot_y = 0.0;
double sensor_rot_z = 0.0;
double sensor_rot_w = 0.0;

// Previous position and orientation values
double prev_sensor_pos_x = 0.0;
double prev_sensor_pos_y = 0.0;
double prev_sensor_pos_z = 0.0;
double prev_sensor_rot_x = 0.0;
double prev_sensor_rot_y = 0.0;
double prev_sensor_rot_z = 0.0;
double prev_sensor_rot_w = 0.0;

// http://ais.informatik.uni-freiburg.de/publications/papers/sittel13tr.pdf
// Previous position and orientation values (leave enough time for calculations to complete)
// derived linear and angular velocity
double lin_vel_x = 0.0;
double lin_vel_y = 0.0;
double lin_vel_z = 0.0;
double ang_vel_p = 0.0;
double ang_vel_q = 0.0;
double ang_vel_r = 0.0;

// Actions
std::string takeoff ("takeoff");


// TODO: I could probably pull this directly from the SDF file, not via head, but maybe dynamically at program start
// TODO: use Eigen/Sparse and it's functions to reduce memory use, most of my matrices are sparse

// environmental variables
double _gravity = 9.81;     // m/s^2

// initialize physical properties of the vehicle
double _mass = 1.5;         // kg
double _length = 0.2555;    // m
static Eigen::Matrix3d _I((Eigen::Matrix3d() << 0.029125, 0.0, 0.0, 0.0, 0.029125, 0.0, 0.0, 0.0, 0.029125).finished());
static Eigen::Matrix3d _I_inv = _I.inverse();

// function declarations
void test_ol_takeoff();
void test_ol_land();
void derived_sensor_values();
void rotor0_cb(MotorSpeedPtr &rotor_vel);
void rotor1_cb(MotorSpeedPtr &rotor_vel);
void rotor2_cb(MotorSpeedPtr &rotor_vel);
void rotor3_cb(MotorSpeedPtr &rotor_vel);

#endif // SET_ROTOR_VEL_HPP
