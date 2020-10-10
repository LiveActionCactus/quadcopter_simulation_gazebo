// Rotor bindings for gazebo simulation; pulls in custom sdf file
// 
// By: Patrick Ledzian
// Date: 01 September 2020
//
// Taken and modified from PX4 v1.9.0
//

// make sure you run this first to set the correct path
// <!-- export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/home/odysseus/projects/inprogress/gazebo_code/iris_testing/build -->

#include "rotor_plugin.hpp"

namespace gazebo
{
double test_size;

RotorModelPlugin::~RotorModelPlugin() {
    updateConnection_->~Connection();
}

void RotorModelPlugin::InitializeParams() {}

// TODO: doesn't seem to be publishing correctly
void RotorModelPlugin::Publish() {
    turning_velocity_msg_.set_data(joint_->GetVelocity(0));

    // TODO: address queue limit issues, not sure if this is going to be a problem
    // This publishes the actual rotation rate of the motors, can subscribe to in in controller code
//    motor_velocity_pub_->Publish(turning_velocity_msg_);

} // end RotorModelPlugin::Publish()

/// \brief The load function is called by Gazebo when the plugin is
/// inserted into simulation
/// \param[in] _model A pointer to the model that this plugin is
/// attached to.
/// \param[in] _sdf A pointer to the plugin's SDF element.
void RotorModelPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
// Store the model pointer for convenience.
model_ = _model;
namespace_.clear();

if (_sdf->HasElement("robotNamespace"))
  namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
else
  gzerr << "[gazebo_motor_model] Please specify a robotNamespace.\n";

node_handle_ = transport::NodePtr(new transport::Node());
node_handle_->Init(namespace_);

if (_sdf->HasElement("jointName"))
  joint_name_ = _sdf->GetElement("jointName")->Get<std::string>();
else
  gzerr << "[gazebo_motor_model] Please specify a jointName, where the rotor is attached.\n";
// Get the pointer to the joint.
joint_ = model_->GetJoint(joint_name_);
if (joint_ == NULL)
  gzthrow("[gazebo_motor_model] Couldn't find specified joint \"" << joint_name_ << "\".");

if (_sdf->HasElement("linkName"))
  link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
else
  gzerr << "[gazebo_motor_model] Please specify a linkName of the rotor.\n";

link_ = model_->GetLink(link_name_);
if (link_ == NULL)
  gzthrow("[gazebo_motor_model] Couldn't find specified link \"" << link_name_ << "\".");

if (_sdf->HasElement("motorNumber"))
    motor_number_ = _sdf->GetElement("motorNumber")->Get<int>();
else
    gzerr << "[gazebo_motor_model] Please specify a motorNumber.\n";

if (_sdf->HasElement("turningDirection")) {
    std::string turning_direction = _sdf->GetElement("turningDirection")->Get<std::string>();
    if (turning_direction == "cw")
        turning_direction_ = turning_direction::CW;
    else if (turning_direction == "ccw")
        turning_direction_ = turning_direction::CCW;
    else
        gzerr << "[gazebo_motor_model] Please only use 'cw' or 'ccw' as turningDirection.\n";
} else {
    gzerr << "[gazebo_motor_model] Please specify a turning direction ('cw' or 'ccw').\n";
}

getSdfParam<std::string>(_sdf, "motorSpeedPubTopic", motor_speed_pub_topic_,
                       motor_speed_pub_topic_);
getSdfParam<double>(_sdf, "rotorDragCoefficient", rotor_drag_coefficient_, rotor_drag_coefficient_);
getSdfParam<double>(_sdf, "rollingMomentCoefficient", rolling_moment_coefficient_,
                  rolling_moment_coefficient_);
getSdfParam<double>(_sdf, "maxRotVelocity", max_rot_velocity_, max_rot_velocity_);
getSdfParam<double>(_sdf, "motorConstant", motor_constant_, motor_constant_);
getSdfParam<double>(_sdf, "momentConstant", moment_constant_, moment_constant_);
getSdfParam<double>(_sdf, "timeConstantUp", time_constant_up_, time_constant_up_);
getSdfParam<double>(_sdf, "timeConstantDown", time_constant_down_, time_constant_down_);
getSdfParam<double>(_sdf, "rotorVelocitySlowdownSim", rotor_velocity_slowdown_sim_, 10);

// This perspective is flipped on it's head. What is published here and in this code should be
// values representative of the vehicle and gazebo. What is subscribed here are the reference values!
updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&RotorModelPlugin::OnUpdate, this, _1));
ref_motor_sub_ = node_handle_->Subscribe<std_msgs::msgs::Float>("~/" + model_->GetName() + "/ref" + motor_speed_pub_topic_, &RotorModelPlugin::RefMotorCallback, this);
motor_velocity_pub_ = node_handle_->Advertise<std_msgs::msgs::Float>("~/" + model_->GetName() + "/sensor" + motor_speed_pub_topic_, 1);

// disturbances
motor_failure_sub_ = node_handle_->Subscribe<msgs::Int>(motor_failure_sub_topic_, &RotorModelPlugin::MotorFailureCallback, this);

// Create the first order filter
rotor_velocity_filter_.reset(new FirstOrderFilter<double>(time_constant_up_, time_constant_down_, ref_motor_rot_vel_));

} // end RotorModelPlugin::Load()


void RotorModelPlugin::OnUpdate(const common::UpdateInfo& _info)
{
    sampling_time_ = _info.simTime.Double() - prev_sim_time_;
    prev_sim_time_ = _info.simTime.Double();
//    UpdateMotorFail();
//    Publish();
    UpdateForcesAndMoments();

} // end RotorModelPlugin::OnUpdate()


// TODO: add clipping for maximum rotor speed (physical limitations)
// TODO: add in PID control of motors instead of instantaneous velocity
void RotorModelPlugin::UpdateForcesAndMoments()
{
    motor_rot_vel_ = ref_motor_vel;

//    if (motor_rot_vel_ / (2 * M_PI) > 1 / (2 * sampling_time_)) {
//        gzerr << "Aliasing on motor [" << motor_number_ << "] might occur. Consider making smaller simulation time steps or raising the rotor_velocity_slowdown_sim_ param.\n";
//    }
//    double real_motor_velocity = motor_rot_vel_ * rotor_velocity_slowdown_sim_;
    double real_motor_velocity = motor_rot_vel_;

    double force = real_motor_velocity * std::abs(real_motor_velocity) * motor_constant_;
    if(!reversible_) {
        // Not allowed to have negative thrust.
        force = std::abs(force);
    }

    // scale down force linearly with forward speed
    // TODO: XXX this has to be modelled better
    //
    ignition::math::Vector3d body_velocity = link_->WorldLinearVel();

    double vel = body_velocity.Length();
    double scalar = 1 - vel / 25.0; // at 50 m/s the rotor will not produce any force anymore
    scalar = ignition::math::clamp(scalar, 0.0, 1.0);
    // Apply a force to the link.
    link_->AddRelativeForce(ignition::math::Vector3d(0, 0, force * scalar));

    // Forces from Philppe Martin's and Erwan Salaün's
    // 2010 IEEE Conference on Robotics and Automation paper
    // The True Role of Accelerometer Feedback in Quadrotor Control
    // - \omega * \lambda_1 * V_A^{\perp}
    ignition::math::Vector3d joint_axis = joint_->GlobalAxis(0);

    ignition::math::Vector3d relative_wind_velocity = body_velocity; // no wind velocity calculation being done "- wind_vel_";
    ignition::math::Vector3d body_velocity_perpendicular = relative_wind_velocity - (relative_wind_velocity.Dot(joint_axis)) * joint_axis;
    ignition::math::Vector3d air_drag = -std::abs(real_motor_velocity) * rotor_drag_coefficient_ * body_velocity_perpendicular;

    // Apply air_drag to link.
    link_->AddForce(air_drag);

    // Moments
    // Getting the parent link, such that the resulting torques can be applied to it.
    physics::Link_V parent_links = link_->GetParentJointsLinks();

    // The transformation from the parent_link to the link_.
    ignition::math::Pose3d pose_difference = link_->WorldCoGPose() - parent_links.at(0)->WorldCoGPose();

    ignition::math::Vector3d drag_torque(0, 0, -turning_direction_ * force * moment_constant_); // neg (-) bc moment is in opposite direction

    // Transforming the drag torque into the parent frame to handle arbitrary rotor orientations.
    ignition::math::Vector3d drag_torque_parent_frame = pose_difference.Rot().RotateVector(drag_torque);
    parent_links.at(0)->AddRelativeTorque(drag_torque_parent_frame);

    ignition::math::Vector3d rolling_moment;

    // - \omega * \mu_1 * V_A^{\perp}
    rolling_moment = -std::abs(real_motor_velocity) * rolling_moment_coefficient_ * body_velocity_perpendicular;
    parent_links.at(0)->AddTorque(rolling_moment);

    // TODO: fix the first order filter, right now it just ouputs 0
    // Apply the filter on the motor's velocity.
    double ref_motor_rot_vel;
    ref_motor_rot_vel = rotor_velocity_filter_->updateFilter(ref_motor_rot_vel_, sampling_time_);           // this is outputting 0 and I don't know why

    // All of the forces have already been applied, this is for visual effect and publishing motor rates
//    std::cout << turning_direction_ << std::endl;
//    std::cout << ref_motor_rot_vel << std::endl;
//    std::cout << rotor_velocity_slowdown_sim_ << std::endl;
//    std::cout << (turning_direction_ * ref_motor_rot_vel / rotor_velocity_slowdown_sim_) << std::endl;
//    std::cout << std::endl;

//    joint_->SetVelocity(0, turning_direction_ * ref_motor_rot_vel / rotor_velocity_slowdown_sim_); // TODO: this is what should be running, but the filter is not working
//        std::cout << real_motor_velocity << std::endl;
//        std::cout << std::endl;
      joint_->SetVelocity(0, turning_direction_ * real_motor_velocity / rotor_velocity_slowdown_sim_ );

} // end RotorModelPlugin::UpdateForcesAndMoments()


double RotorModelPlugin::MapEscToMotor()
{
  // TODO: build out this function. I need a way to generate the mapping, 
  // TODO: vary the mapping wrt battery voltage
  ;
} // end RotorModelPlugin::MapEscToMotor()


// TODO: need to do better here; if a motor fails, then what?
void RotorModelPlugin::UpdateMotorFail()
{
    if (motor_number_ == motor_Failure_Number_ - 1){
        // motor_constant_ = 0.0;
        joint_->SetVelocity(0,0);
        if (screen_msg_flag){
            std::cout << "Motor number [" << motor_Failure_Number_ <<"] failed!  [Motor thrust = 0]" << std::endl;
            tmp_motor_num = motor_Failure_Number_;

            screen_msg_flag = 0;
        }
    }else if (motor_Failure_Number_ == 0 && motor_number_ ==  tmp_motor_num - 1){
        if (!screen_msg_flag){
            //motor_constant_ = kDefaultMotorConstant;
            std::cout << "Motor number [" << tmp_motor_num <<"] running! [Motor thrust = (default)]" << std::endl;
            screen_msg_flag = 1;
        }
    }
} // end RotorModelPlugin::UpdateMotorFail()

// TODO: set motor failure sensing in control code; then publish here if there is a failure
void RotorModelPlugin::MotorFailureCallback(const boost::shared_ptr<const msgs::Int> &fail_msg)
{
    motor_Failure_Number_ = fail_msg->data();
}

// TODO: clip based on max rotational velocity
void RotorModelPlugin::RefMotorCallback(const boost::shared_ptr<const std_msgs::msgs::Float> &ref_motor_vel_update)
{
    ref_motor_vel = ref_motor_vel_update->data();
//    std::cout "in the motor callback: " << ref_motor_vel << std::endl;
}

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(RotorModelPlugin)

} // end RotorModelPlugin::RefMotorCallback()