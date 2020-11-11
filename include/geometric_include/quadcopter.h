// Header file for quadcopter.cpp
//
// By: Patrick Ledzian
// Date: 11 November 2020
//

#ifndef GAZEBOVEHICLESIMBINDINGS_QUADCOPTER_H
#define GAZEBOVEHICLESIMBINDINGS_QUADCOPTER_H

#include <eigen3/Eigen/Eigen>
#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include "../../msgs/include/Float.pb.h"
#include "../../msgs/include/local_poses_stamped.pb.h"

class Uncopyable {
protected:
    Uncopyable() {}
    ~Uncopyable() {}

private:
    Uncopyable(const Uncopyable&);              // prevent copy constructor and copy assignment operator
    Uncopyable& operator=(const Uncopyable&);
};

// TODO: add logging of state variables
class Quadcopter: private Uncopyable{
public:
    Quadcopter();
    ~Quadcopter();
    void update_state_and_data();      // default is disarmed

private:
    //// gazebo variables
    gazebo::transport::NodePtr node_handle_;         // TODO:: maybe "Node" vs "NodePtr"; different errors
    typedef const boost::shared_ptr<const gazebo::msgs::LocalPosesStamped> ConstLocalPosesStampedPtr;
    gazebo::transport::SubscriberPtr sub;
    gazebo::transport::PublisherPtr pub0;
    gazebo::transport::PublisherPtr pub1;
    gazebo::transport::PublisherPtr pub2;
    gazebo::transport::PublisherPtr pub3;

    //// time variables
    double sim_time_;
    double prev_sim_time_;           // can I just make this a static variable somewhere?
    double sim_time_delta_;          // TODO: put this as a local variable in derived sensor values function

    //// state variable
    int sim_state_;                  // 0 - not armed, 1 - pre-armed, 2 - armed

    //// publish variables
    std_msgs::msgs::Float ref_motor_vel0_;
    std_msgs::msgs::Float ref_motor_vel1_;
    std_msgs::msgs::Float ref_motor_vel2_;
    std_msgs::msgs::Float ref_motor_vel3_;

    //// physical properties
    double gravity_;                        // m/s^2
    double mass_;                           // kg
    double hover_point_;                    // rad/s for one motor only
    double motor_force_const_;
    Eigen::Matrix4d motor_mapping_;

    //// measured position and orientation
    Eigen::Matrix<double,1,3> sensor_pos_;                  // m
    Eigen::Matrix<double,1,3> prev_sensor_pos_;
    Eigen::Matrix<double,1,3> derived_lin_vel_;             // m/s
    Eigen::Matrix<double,1,4> sensor_quat_;
    Eigen::Matrix<double,1,3> derived_euler_att_;           // rad
    Eigen::Matrix<double,1,3> prev_derived_euler_att_;
    Eigen::Matrix<double,1,3> derived_pqr_att_;             // rad/s

    //// desired position and orientation
    Eigen::Matrix<double,1,3> desired_pos_;                 // m
    Eigen::Matrix<double,1,3> desired_vel_;                 // m/s
    Eigen::Matrix<double,1,3> desired_acc_;                 // m/s^{2}
    Eigen::Matrix<double,1,3> desired_euler_att_;           // rad
    Eigen::Matrix<double,1,3> desired_pqr_att_;             // rad/s
    Eigen::Matrix<double,1,4> desired_thrust_;              // rad/s

    //// helper variables
    Eigen::Matrix<double,1,3> final_att_deltas_;
    Eigen::Matrix<double,1,4> desired_rotor_rates_;         // rad/s

    //// function definitions
    void local_poses_cb(ConstLocalPosesStampedPtr &local_pose);
    void derived_sensor_values();
    void publish_rotor_cmds();

    //// helper function definitions
    // TODO: not sure if static function definitions cause issues with concurrency / multi-threading
    static Eigen::Matrix<double,1,3> quat2euler(const Eigen::Ref<const Eigen::Matrix<double,1,4>>& q_);
    static Eigen::Matrix<double,3,3> quat2rot(const Eigen::Ref<const Eigen::Matrix<double,1,4>>& q_);
    static Eigen::Matrix<double,1,3> derive_ang_velocity(const Eigen::Ref<const Eigen::Matrix<double,1,3>>& e_,
                                                         const Eigen::Ref<const Eigen::Matrix<double,1,3>>& prev_e_);
};

#endif //GAZEBOVEHICLESIMBINDINGS_QUADCOPTER_H
