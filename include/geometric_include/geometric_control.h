//
// Created by odysseus on 11/8/20.
//

#ifndef GAZEBOVEHICLESIMBINDINGS_GEOMETRIC_CONTROL_H
#define GAZEBOVEHICLESIMBINDINGS_GEOMETRIC_CONTROL_H

#include <eigen3/Eigen/Eigen>
#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include "../../msgs/include/Float.pb.h"
#include "../../msgs/include/local_poses_stamped.pb.h"


//// Global Gazebo variables
//gazebo::transport::NodePtr _node_handle;
typedef const boost::shared_ptr<const gazebo::msgs::LocalPosesStamped> ConstLocalPosesStampedPtr;
void local_poses_cb(ConstLocalPosesStampedPtr &local_pose);

class Uncopyable {
protected:
    Uncopyable() {}
    ~Uncopyable() {}

private:
    Uncopyable(const Uncopyable&);              // prevent copy constructor and copy assignment operator
    Uncopyable& operator=(const Uncopyable&);
};

class Quadcopter: private Uncopyable{
public:
    Quadcopter();
//    sub = _node_handle->Subscribe("~/pose/local/info", local_poses_cb);
//    void update_sensor_data(ConstLocalPosesStampedPtr &local_pose);

    // prevent copy assignment

private:
    //// gazebo variables
//    gazebo::transport::NodePtr node_handle_;         // TODO:: maybe "Node" vs "NodePtr"
//    gazebo::transport::Subscriber sub;
//    typedef const boost::shared_ptr<const gazebo::msgs::LocalPosesStamped> ConstLocalPosesStampedPtr;
//    const gazebo::transport::SubscriberPtr sub;

    // make const pointers to quadcopter characteristics; these will be initialized in the constructor
    //// time variables
    double sim_time_;
    double prev_sim_time_;           // can I just make this a static variable somewhere?
//    double sim_time_delta;          // put this as a local variable in derived sensor values function

    //// state variable
    int sim_state_;

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
    Eigen::Matrix3d q_hat_;
    Eigen::Matrix<double,1,4> quat_normalized_;
    Eigen::Matrix<double,1,3> final_att_deltas_;
};

class Trajectory: Quadcopter{
public:
    Trajectory();           // need to specify the trajectory chosen here; can inherit desired position from Quadcopter; need to do some error checking though

private:
    // trajectory specific data; stats; optimization algorithm
    //// different trajectories to choose from
    std::string steps_;
    std::string hover_;
    std::string min_snap_;
    std::string circle_;
    std::string figure_eight_;

    //// setpoints for step response trajectory (maybe put this in "Controller" and have that inherit from Trajectory)
    bool set_pt1_ = 1;
    bool set_pt2_ = 0;
    bool set_pt3_ = 0;
    bool set_pt4_ = 0;

};

class Controller: Trajectory{
public:
    Controller();
    // different position and attitude controllers (could create position and attitude subclasses)

private:
    // controller specific data/properties; bounds checking (geometric modes; ctrl gains)
    //// controller gains
    static Eigen::Matrix<double,1,3> Kp_pos_;
    static Eigen::Matrix<double,1,3> Kd_pos_;
    static Eigen::Matrix<double,1,3> Kp_ang_;
    static Eigen::Matrix<double,1,3> Kd_ang_;
};


#endif //GAZEBOVEHICLESIMBINDINGS_GEOMETRIC_CONTROL_H
