// Rotor bindings header file, binds with gazebo simulation environment
//
// By: Patrick Ledzian
// Date: 01 September 2020
//
// Taken and modified from PX4 v1.9.0
//

// make sure you run this first to set the correct path
// <!-- export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/home/odysseus/projects/inprogress/gazebo_code/iris_testing/build -->

// Style guide:
// test  -- a variable defined local to a function
// _test -- a parameter of a function
// test_ -- a parameter defined in a header file and living in the namespace

#ifndef ROTOR_PLUGIN_HPP
#define ROTOR_PLUGIN_HPP

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>
#include <boost/bind.hpp>
#include <gazebo/msgs/msgs.hh>

#include "common.hpp"
#include "motor_model.hpp"
#include "../msgs/include/Float.pb.h"
#include "../msgs/include/MotorSpeed.pb.h"

// I can include the already compiled protobuf files from Gazebo's source
// #include <gazebo/msgs/wind.pb.h>

namespace turning_direction {
	const static int CCW = 1;
	const static int CW = -1;
}

namespace gazebo {
// Default values
static const std::string kDefaultMotorFailureNumSubTopic = "/gazebo/motor_failure_num";
static const std::string kDefaultMotorVelocityPubTopic = "/motor_speed";

// Protobuf test
typedef const boost::shared_ptr<const mav_msgs::msgs::MotorSpeed> MotorSpeedPtr;
static const std::string kDefaultMotorTestSubTopic = "motors";

// Set the max_force_ to the max double value. The limitations get handled by the FirstOrderFilter.
static constexpr double kDefaultMaxForce = std::numeric_limits<double>::max();
static constexpr double kDefaultMotorConstant = 8.54858e-06;
static constexpr double kDefaultMomentConstant = 0.016;
static constexpr double kDefaultTimeConstantUp = 1.0 / 80.0;
static constexpr double kDefaultTimeConstantDown = 1.0 / 40.0;
static constexpr double kDefaultMaxRotVelocity = 838.0;
static constexpr double kDefaultRotorDragCoefficient = 1.0e-4;
static constexpr double kDefaultRollingMomentCoefficient = 1.0e-6;
static constexpr double kDefaultRotorVelocitySlowdownSim = 10.0;

/// \brief A plugin to control a quadcopter rotor.
class RotorModelPlugin : public ModelPlugin, public MotorModel {
public:
    RotorModelPlugin()
            : ModelPlugin(),
              MotorModel(),
              motor_failure_sub_topic_(kDefaultMotorFailureNumSubTopic),
              motor_speed_pub_topic_(kDefaultMotorVelocityPubTopic),
              motor_number_(0),
              motor_Failure_Number_(0),
              turning_direction_(turning_direction::CW),
              max_rot_velocity_(kDefaultMaxRotVelocity),
              moment_constant_(kDefaultMomentConstant),
              motor_constant_(kDefaultMotorConstant),
//              motor_test_sub_topic_(kDefaultMotorTestSubTopic),
              ref_motor_rot_vel_(0.0),
              rolling_moment_coefficient_(kDefaultRollingMomentCoefficient),
              rotor_drag_coefficient_(kDefaultRotorDragCoefficient),
              rotor_velocity_slowdown_sim_(kDefaultRotorVelocitySlowdownSim),
              time_constant_down_(kDefaultTimeConstantDown),
              time_constant_up_(kDefaultTimeConstantUp),
              reversible_(false) {
    }

	virtual ~RotorModelPlugin();

    virtual void InitializeParams();
    virtual void Publish();

protected:
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    virtual void OnUpdate(const common::UpdateInfo & /*_info*/);
    virtual void UpdateForcesAndMoments();
    virtual void UpdateMotorFail();
    virtual double MapEscToMotor();

private:
    std::string motor_failure_sub_topic_;
    std::string joint_name_;
    std::string link_name_;
    std::string motor_speed_pub_topic_;
    std::string namespace_;

    physics::ModelPtr model_;
    physics::JointPtr joint_;
    physics::LinkPtr link_;

    common::PID pid_;

    transport::NodePtr node_handle_;
    transport::PublisherPtr motor_velocity_pub_;
    transport::SubscriberPtr motor_failure_sub_;
    transport::SubscriberPtr ref_motor_sub_;

    event::ConnectionPtr updateConnection_;

    std_msgs::msgs::Float turning_velocity_msg_;

    int motor_number_;
    int turning_direction_;
    int motor_Failure_Number_; /*!< motor_Failure_Number is (motor_number_ + 1) as (0) is considered no_fail. Publish accordingly */
    int screen_msg_flag = 1;
    int tmp_motor_num; // A temporary variable used to print msg

    double max_rot_velocity_;
    double moment_constant_;
    double motor_constant_;
    double ref_motor_rot_vel_;
    double rolling_moment_coefficient_;
    double rotor_drag_coefficient_;
    double rotor_velocity_slowdown_sim_;
    double time_constant_down_;
    double time_constant_up_;

    float ref_motor_vel = 0.0;

    bool reversible_;

    void MotorFailureCallback(const boost::shared_ptr<const msgs::Int> &fail_msg);  /*!< Callback for the motor_failure_sub_ subscriber */
    std::unique_ptr<FirstOrderFilter<double>>  rotor_velocity_filter_;
    void RefMotorCallback(const boost::shared_ptr<const std_msgs::msgs::Float> &ref_motor_vel_update);

}; // end class RotorModelPlugin
} // end namespace gazebo

#endif //ROTOR_PLUGIN_HPP
