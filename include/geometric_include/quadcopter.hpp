// Header file for quadcopter.cpp
//
// By: Patrick Ledzian
// Date: 11 November 2020
//

#ifndef GAZEBOVEHICLESIMBINDINGS_QUADCOPTER_HPP
#define GAZEBOVEHICLESIMBINDINGS_QUADCOPTER_HPP

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
    void run();

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
    double motor_torque_const_;
    double arm_length_;
    Eigen::Matrix<double,4,4> motor_force_mapping_;
    Eigen::Matrix<double,4,4> inv_motor_force_mapping_;
    Eigen::Matrix<double,3,3> J_;           // vehicle inertial tensor          // TODO: does this make the controller sensitive to errors in J_??

    //// measured position and orientation
    Eigen::Matrix<double,1,3> sensor_pos_;                  // m
    Eigen::Matrix<double,1,3> prev_sensor_pos_;
    Eigen::Matrix<double,1,3> derived_lin_vel_;             // m/s
    Eigen::Matrix<double,3,3> derived_rot_;
    Eigen::Matrix<double,1,4> sensor_quat_;
    Eigen::Matrix<double,1,3> derived_euler_att_;           // rad
    Eigen::Matrix<double,1,3> prev_derived_euler_att_;
    Eigen::Matrix<double,1,3> derived_pqr_att_;             // rad/s

    //// desired position and orientation
//    Eigen::Matrix<double,1,3> desired_pos_;                 // m
//    Eigen::Matrix<double,1,3> desired_vel_;                 // m/s
//    Eigen::Matrix<double,1,3> desired_acc_;                 // m/s^{2}
//    Eigen::Matrix<double,1,3> desired_euler_att_;           // rad
//    Eigen::Matrix<double,1,3> desired_pqr_att_;             // rad/s
//    Eigen::Matrix<double,1,4> desired_thrust_;              // rad/s

    //// helper variables
    Eigen::Matrix<double,1,3> final_att_deltas_;

    //// function definitions
    void local_poses_cb(ConstLocalPosesStampedPtr &local_pose);
    void update_state_and_data();      // default is disarmed
    void derived_sensor_values();
    void publish_rotor_cmds();

    //// helper function definitions
    // TODO: not sure if static function definitions cause issues with concurrency / multi-threading
    static Eigen::Matrix<double,1,3> quat2euler(const Eigen::Ref<const Eigen::Matrix<double,1,4>>& q_);
    static Eigen::Matrix<double,3,3> quat2rot(const Eigen::Ref<const Eigen::Matrix<double,1,4>>& q_);
    static Eigen::Matrix<double,1,3> derive_ang_velocity(const Eigen::Ref<const Eigen::Matrix<double,1,3>>& e_,
                                                         const Eigen::Ref<const Eigen::Matrix<double,1,3>>& prev_e_);

    //// nested class declarations
    class Trajectory {
    public:
        Trajectory(std::string traj_ = "none");           // need to specify the trajectory chosen here; can inherit desired position from Quadcopter; need to do some error checking though
        void run_trajectory_update();
        void set_new_trajectory(std::string new_traj_);
        Eigen::Matrix<double,1,3> get_desired_pos();             // TODO: make "desired_lin" into a struct of pos, vel, acc
        Eigen::Matrix<double,1,3> get_desired_vel();
        Eigen::Matrix<double,1,3> get_desired_acc();

    private:
        // trajectory specific data; stats; optimization algorithm
        //// different trajectories to choose from
        std::string hover_;
        std::string steps_;
        std::string min_snap_;
        std::string circle_;
        std::string figure_eight_;

        //// setpoints for step response trajectory (maybe put this in "Controller" and have that inherit from Trajectory)
        bool set_pt1_ = 1;
        bool set_pt2_ = 0;
        bool set_pt3_ = 0;
        bool set_pt4_ = 0;

        //// desired values to achieve
        std::string desired_traj_;
        Eigen::Matrix<double, 1, 3> desired_pos_;
        Eigen::Matrix<double, 1, 3> desired_vel_;
        Eigen::Matrix<double, 1, 3> desired_acc_;
        Eigen::Matrix<double, 1, 3> desired_euler_att_;
        Eigen::Matrix<double, 1, 3> desired_pqr_att_;

        //// function definitions
        void basic_hover();

    }; // end class Trajectory

    class Controller {
//    Quadcopter *quad;

    public:
        Controller();
        void position_control(Quadcopter &q, Trajectory &t);
        void attitude_control(Quadcopter &q, Trajectory &t);
        void set_rotor_rates(Quadcopter &q);

    private:
        // controller specific data/properties; bounds checking (geometric modes; ctrl gains)
        //// controller gains
//        Eigen::Matrix<double, 1, 3> Kpos_;                // I'm assigning the diagonal elements the same...
//        Eigen::Matrix<double, 1, 3> Kvel_;                // TODO: come back here and implement the matrix version
//        Eigen::Matrix<double, 1, 3> Krot_;
//        Eigen::Matrix<double, 1, 3> Kang_;
        double Kpos_;
        double Kvel_;
        double Krot_;
        double Kang_;

        //// error vectors
        Eigen::Matrix<double,1,3> pos_err_;
        Eigen::Matrix<double,1,3> vel_err_;
        Eigen::Matrix<double,1,3> rot_err_;
        Eigen::Matrix<double,1,3> ang_vel_err_;

        //// inherent properties
        double pos_time_;
        double att_time_;
        double pos_time_loop_;
        double att_time_loop_;

        //// derived values
        Eigen::Matrix<double,3,3> Rc_;
        Eigen::Matrix<double,3,3> Rc_prev_;
        Eigen::Matrix<double,1,3> omegac_;
        Eigen::Matrix<double,1,3> omegac_prev_;

        //// desired values
        Eigen::Matrix<double,1,4> desired_rotor_forces_;        // N?
        Eigen::Matrix<double,1,4> desired_rotor_rates_;         // rad/s
        double desired_thrust_magnitude_;                       // sum of rotor forces
        Eigen::Matrix<double,1,3> desired_moments_;

        //// helper function definitions
//        void set_rotor_rates(Quadcopter &q);

    }; // end class Controller

    //// nested class objects
    Trajectory trajectory;
    Controller controller;

}; // end class Quadcopter

#endif //GAZEBOVEHICLESIMBINDINGS_QUADCOPTER_HPP
