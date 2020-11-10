// Hover envelope control of quadcopter with bindings to the Gazebo simulation environment
//
// By: Patrick Ledzian
// Date: 01 October 2020
//

#include "../../include/hover_env_include/hover_env_control.hpp"

// Eigen cheatsheets
// https://gist.github.com/gocarlos/c91237b02c120c6319612e42fa196d77
// http://dev.ipol.im/~yiqing/vj_old/vj/eigen/doc/AsciiQuickReference.txt
// https://igl.ethz.ch/projects/libigl/matlab-to-eigen.html
//

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"

// TODO: not sure if "iris" is the same index every run, need to run a for loop to find it and its index
// TODO: put in write lock here and read lock in derived_sensor_values()

//// Subscriber callback function
// pulls from Gazebo: sim time (seconds), linear position from origin (meters), and orientation (quaternion)
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

} // end local_poses_cb()

//// Main loop
// First portion sets up the messaging with Gazebo and logging file handle
// Second portion is the while(1) loop that runs for duration of quadcopter manuevers
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

    // Subscribe to measured position and orientation values
    gazebo::transport::SubscriberPtr sub5 = node_handle->Subscribe("~/pose/local/info", local_poses_cb);

    // TODO: organized this so the initial state comes from read-in values
    initialize_variables();                         // assumes starting from origin with zeroed orientation

    // open file handle for storing test data for later analysis
    std::ofstream testdata;
    testdata.open("test_data.txt");
    u_int64_t ctr = 0;                  // timing counter, good for 2^64 bits (essentially infinite)

    //// Main loop
    // runs all of the quadcopter functions and maneuvers
    while(1)
    {
        //// Basic state machine
        // states:
        //  0 -- disarmed; no motor commands
        //  1 -- pre-arm; spin the motors at a slow speed, runs all sensor derivations
        //  2 -- arm; full functionality, quadcopter is moving
        if(sim_state == 0)
        {
            std::cout << "Initializing" << std::endl;
            while(sim_time < 6.0){
                ;       // wait for Gazebo
            }
            // Initialize the quadcopter variables
            prev_sim_time = sim_time;
            prev_sim_time_pos = sim_time;
            prev_sim_time_att = sim_time;
            sim_state = 2;
            std::cout << "Armed" << std::endl;
        } else if(sim_state == 2) {
            derived_sensor_values();                        // called before a control decision; derive velocities

            // higher-level trajectory loop
            if ((sim_time - prev_sim_time_pos) > 0.01) {
                // commanded actions
                if (hover == _argv[1]) {
                    basic_hover();

                } else if (steps == _argv[1]) {
                    setpoint_trajectory();

                } else if (min_snap == _argv[1]) {
                    if (!_start_traj & ((_desired_pos - _sensor_pos).lpNorm<2>() < 0.1)) {
                        if ((_desired_pos - _sensor_pos).lpNorm<2>() < 0.1) {
                            _start_traj = 1;        // ensures vehicle is at hover first
                        }
                    } else {
                        if(!_traj_finished) {
                            minimum_snap_trajectory();
                        } else {
                            // conclude the trajectory by hovering in place
                            _desired_pos << _traj_setpoints(_traj_setpoints.rows()-1, 0), _traj_setpoints(_traj_setpoints.rows()-1, 1),
                                                _traj_setpoints(_traj_setpoints.rows()-1, 2);
                            _start_traj = 0;
                        }
                    }
                } else if(_circle == _argv[1]){
                    circling_trajectory();
                } else if(_figure_eight == _argv[1]){
                    figure_eight_trajectory();
                }else {
                    test_ol_land();
                } // end if hover, steps, or min_snap trajectories
            } // end trajectory loop

            // position controller loop
            if ((sim_time - prev_sim_time_pos) > 0.01) {
                basic_position_controller();
                prev_sim_time_pos = sim_time;
            }

            // attitude controller loop
            if ((sim_time - prev_sim_time_att) > 0.001) {
                basic_attitude_controller();
                prev_sim_time_att = sim_time;
            }

            test_cl_takeoff();          // write derived motor commands to the message template

            // log data for later analysis
            ctr++;          // iterate the logging counter
            if ((ctr % 1000) == 0) {
                testdata << sim_time << "\n";
                testdata << std::fixed << std::setprecision(8) << _desired_pos << "\n";
                testdata << std::fixed << std::setprecision(8) << _sensor_pos << "\n";
                testdata << std::fixed << std::setprecision(8) << _desired_euler_att << "\n";
                testdata << std::fixed << std::setprecision(8) << _derived_euler_att << "\n";
                testdata << std::fixed << std::setprecision(8) << _desired_thrust << "\n";
                testdata << std::fixed << std::setprecision(8) << _final_att_deltas << "\n";

            } // end if((ctr % 1000) == 0)

            // publish rotor commands
            pub0->Publish(ref_motor_vel0);
            pub1->Publish(ref_motor_vel1);
            pub2->Publish(ref_motor_vel2);
            pub3->Publish(ref_motor_vel3);

        } // end if(sim_state == 2)
    } // end while(1)
} // end main()

//// Basic takeoff and hover
// gets the quadcopter hovering at a stable state
void basic_hover()
{
    _desired_pos << 0.0, 0.0, 2.0;
}

//// Various step responses
// 1m sized step responses to test controller response in the x, y, z directions
void setpoint_trajectory()
{
    if(((_desired_pos - _sensor_pos).lpNorm<2>() < 0.01) & (_set_pt1 == 1))
    {
        _desired_pos << 1.0, 0.0, 2.0;
        std::cout << "New desired position: " << _desired_pos << std::endl;
        _set_pt1 = 0;
        _set_pt2 = 1;

    } else if (((_desired_pos - _sensor_pos).lpNorm<2>() < 0.01) & (_set_pt2 == 1))
    {
        _desired_pos << 1.0, 1.0, 2.0;
        std::cout << "New desired position: " << _desired_pos << std::endl;
        _set_pt2 = 0;
        _set_pt3 = 1;
    } else if (((_desired_pos - _sensor_pos).lpNorm<2>() < 0.01) & (_set_pt3 == 1))
    {
        _desired_pos << 1.0, 1.0, 3.0;
        std::cout << "New desired position: " << _desired_pos << std::endl;
        _set_pt3 = 0;
        _set_pt4 = 1;
    } else if (((_desired_pos - _sensor_pos).lpNorm<2>() < 0.01) & (_set_pt4 == 1))
    {
        _desired_pos << 1.0, 1.0, 2.0;
        std::cout << "New desired position: " << _desired_pos << std::endl;
        _set_pt4 = 0;
    }
} // end setpoint_trajectory()

//// Basic continuous circling trajectory
// circles around the origin (0.0, 0.0) at a constant altitude
void circling_trajectory()
{
    // References:
    // https://gamedev.stackexchange.com/questions/9607/moving-an-object-in-a-circular-path#:~:text=You%20can%20do%20that%20using,radius%20is%20its%20radius.
    // https://math.stackexchange.com/questions/26329/formula-to-move-the-object-in-circular-path
    // Fast Nonlinear Model Predictive Control for Multicopter Attitude Tracking on SO(3)

    // TODO: have the yaw vector always pointing to the origin, maybe by transforming the euler att to world frame?
    // TODO: either fix the random spikes in roll/pitch/yaw derived measurements or filter them
    // TODO: figure out how to generate a constant desired velocity to be tracked with this trajectory method

    double radius_ = 2.0;          // circle radius
    if(!_start_traj) {
        _desired_pos << 1.0, 0.0, _desired_pos(2);
    }

    if(!_start_traj & ((_desired_pos - _sensor_pos).lpNorm<2>() < 0.01))
    {
        std::cout << "Starting Circling Trajectory... " << std::endl;
        _start_traj = 1;
        _traj_start_time = sim_time;
    } else if(_start_traj){
        _traj_time = sim_time - _traj_start_time;
        _desired_pos << radius_*cos(_traj_time), radius_*sin(_traj_time), _desired_pos(2);
        _desired_vel << -radius_*sin(_traj_time), radius_*cos(_traj_time), 0.0;
        _desired_acc << -radius_*cos(_traj_time), -radius_*sin(_traj_time), 0.0;
    }
}

//// Generates a Gerono Lemniscate trajectory for tracking
// centered on the origin at (0.0, 0.0)
void figure_eight_trajectory() {
    // References
    // Differential Flatness of Quadrotor Dynamics Subject to Rotor Drag for Accurate Tracking of High-Speed Trajectories
    //
    // Also called: the Gerono Lemniscate trajectory
    //

    double pos_scaling_ = 2.5;          // larger scales the trajectory up in size
    double time_scale_ = 0.9;           // lower is slower (all the way down to 0.0)

    if (!_start_traj) {
        _desired_pos << 1.0, 1.0, _desired_pos(2);
    }

    if (!_start_traj & ((_desired_pos - _sensor_pos).lpNorm<2>() < 0.01)) {
        std::cout << "Starting Figure-8 Trajectory... " << std::endl;
        _start_traj = 1;
        _traj_start_time = sim_time;
    } else if (_start_traj) {
        _traj_time = sim_time - _traj_start_time;
        _desired_pos << pos_scaling_ * cos(time_scale_*_traj_time), pos_scaling_ * sin(time_scale_*_traj_time) * cos(time_scale_*_traj_time), _desired_pos(2);
        _desired_vel << -pos_scaling_*time_scale_ * sin(time_scale_*_traj_time),
                pos_scaling_*time_scale_ * (pow(cos(time_scale_*_traj_time), 2) - pow(sin(time_scale_*_traj_time), 2)), 0.0;
        _desired_acc << -pos_scaling_*time_scale_*time_scale_ * cos(time_scale_*_traj_time),
                            -pos_scaling_*time_scale_*time_scale_ * 4.0* sin(time_scale_*_traj_time)*cos(time_scale_*_traj_time), 0.0;
    }
} // end figure_eight_trajectory()

//// Generate minimum snap trajectory
// produces an array of nx9 states to track minimizing the snap of the linear movements
// TODO: the trajectory generator really doesn't like negative values for waypoints... can I fix this?
void minimum_snap_trajectory()
{
    // References:
    // Minimum Snap Trajectory Generation and Control for Quadrotors
    //

    Eigen::Matrix<double, 1, 8> temp_;          // store desired position, velocity, and acceleration calc before assignment

    if(_is_optimized == 0)
    {
        // Waypoints set in initialize_variables()
        generate_ts();          // stores the time-series goals in _ts
        min_snap_optimization();
        _is_optimized = 1;

    } else if(_start_traj == 0) {
        ;                               // wait until trajectory call begins
        _traj_start_time = sim_time;
    } else if(_start_traj == 1)
    {
        _traj_time = sim_time - _traj_start_time;   // track trajectory specific time

        int idx_ = 0;                    // _ts index of current operation

        // search the trajectory time-series to find the current index
        for(int i = 0; i < _ts.rows(); i++)
        {
            if(_traj_time < _ts(i)){
                idx_ = i;
                break;
            } else if ((_ts.rows()-1) == i) {
                idx_ = i;       // capture final case
            }
        } // end time series search

        // seems to work better when I do this; captures the 0 case
        if(idx_ == 0){
            idx_ = 0;
        } else {
            idx_ = idx_ - 1;
        } // end index correction and error catching

        // check if this is the last index and if the quadcopter is there; then finish
        if(((idx_+2) == _ts.rows()) & ((_traj_setpoints.block(idx_+1, 0, 1, 3) - _sensor_pos).lpNorm<2>() < 0.2))
        {
            std::cout << std::endl;
            std::cout << "*** Trajectory finished *** " << std::endl;
            std::cout << "Desired finish time: " << _ts(idx_+1) << std::endl;
            std::cout << "Actual finish time: " << _traj_time << std::endl;
            _traj_finished = 1;
        }

        // Apply optimized coefficients to the setpoints
        // Generate desired position
        temp_ << std::pow(_traj_time, 7), std::pow(_traj_time, 6), std::pow(_traj_time, 5), std::pow(_traj_time, 4),
                    std::pow(_traj_time, 3), std::pow(_traj_time, 2), _traj_time, 1.0;
        _desired_pos = temp_ * _coef.block(8*idx_, 0, 8, 3);

        // Generate desired velocity
        temp_ << 7.0 * std::pow(_traj_time, 6), 6.0 * std::pow(_traj_time, 5), 5.0 * std::pow(_traj_time, 4),
                    4.0 * std::pow(_traj_time, 3), 3.0 * std::pow(_traj_time, 2), 2.0 * _traj_time, 1.0, 0.0;
        _desired_vel = temp_ * _coef.block(8*idx_, 0, 8, 3);

        // Generate desired acceleration
        temp_ << 42.0 * std::pow(_traj_time, 5), 30.0 * std::pow(_traj_time, 4), 20.0 * std::pow(_traj_time, 3),
                    12.0 * std::pow(_traj_time, 2), 6.0 * _traj_time, 2.0, 0.0, 0.0;
        _desired_acc = temp_ * _coef.block(8*idx_, 0, 8, 3);

    } // end if
} // end minimum_snap_trajectory()

//// Generate time-series for the minimum snap trajectory
// produces a nx1 array of times which the vehicle needs to waypoint
void generate_ts()
{
    double speed_ = 1.75;         // m/s                    // 1.75 m/s is pretty much the limit with hover-envelope, final error increased to 0.2m
    int _path_max_size = _traj_setpoints.rows();
    double path_len_;
    path_len_ = (_traj_setpoints.middleRows(1, _path_max_size-1)
            - _traj_setpoints.middleRows(0, _path_max_size-1)).array().pow(2).rowwise().sum().cwiseSqrt().sum();
    _total_traj_time = path_len_/speed_;         // based on naive path length
    Eigen::MatrixXd path_seg_lenth_((_path_max_size-1),1);
    path_seg_lenth_ = (_traj_setpoints.middleRows(1, _path_max_size-1)
                         - _traj_setpoints.middleRows(0, _path_max_size-1)).array().pow(2).rowwise().sum().cwiseSqrt().cwiseSqrt();

    double cumsum_ = 0;
    for(int i=0; i<(_path_max_size-1); i++)
    {
        _ts(i, 0) = path_seg_lenth_(i) + cumsum_;
        cumsum_ += path_seg_lenth_(i,0);
    }

    double scaling_ = _ts(_path_max_size-2, 0);
    _ts = _ts.array() / scaling_;       // scaled/normalized time series

    Eigen::MatrixXd prev_ts_(_ts.rows(), _ts.cols());
    prev_ts_ = _ts;                     // store previous ts to add 0.0 at the start

    _ts.resize(_path_max_size, 1);
    _ts << 0.0, prev_ts_;               // add in 0.0 seconds for first time

    _ts = _ts * _total_traj_time;      // final goal times for each waypoint (to be shifted later)

} // end generate_ts()

//// Minimum snap trajectory optimization
// find the optimizing coefficients for the trajectory
void min_snap_optimization()
{
    int m_ = _traj_setpoints.rows();            // wpts
    int n_ = _traj_setpoints.cols();            // x,y,z
    m_ = m_ - 1;                                // mathematical convenience         // TODO: I think this is an error; cuts off last trajectory

    //    double eps_ = 2e-52;                        // TODO: can I use float here? if not... then limits?
    double eps_ = 2e-10;
    _coef.resize(8*m_, 3);            // needs to match setpoints * constraints dimensions

    // define template matrices
    Eigen::MatrixXd X_(8*m_, n_);
    Eigen::MatrixXd Y_(8*m_, n_);
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A_;
    A_.resize(8*m_, 8*m_);

    // https://stackoverflow.com/questions/31159196/can-we-create-a-vector-of-eigen-matrix
    std::vector<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> > A_vec_;

    // zero the matrices
    X_.setZero();
    Y_.setZero();
    A_.setZero();
    A_ = A_.setIdentity();
    A_ = A_ * eps_;                 // condition the matix so inversions aren't singular

//    m_ = m_ - 1;                                // mathematical convenience         // TODO: I think this is an error; cuts off last trajectory

    // initialize vector of A_ matrices
    for(int i = 0; i < n_; i++)
    {
        A_vec_.push_back(A_);
//        std::cout << A_vec_[i] << std::endl;
//        std::cout << A_vec_[i](0,0) << std::endl;
//        std::cout << std::endl;
    }

    // In a 7th order minimum-snap trajectory there are 8 parameters for each subpath
    for(int i = 0; i < n_; i++)
    {
        int idx_ = 0;
        Eigen::Matrix<double, 1, 8> temp_;
//        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> temp_;
//        temp_.resize(1, 8);         // this size stays fairly consistent

        // Constraint 1: x_k(t_k) = x_{k+1}(t_k) = p_k, where p_k is a waypoint
        for(int k = 0; k < m_-1; k++)
        {
            // A[idx, 8*k:8*(k+1), i]
            temp_ << std::pow(_ts(k+1), 7), std::pow(_ts(k+1), 6), std::pow(_ts(k+1), 5), std::pow(_ts(k+1), 4),
                        std::pow(_ts(k+1), 3), std::pow(_ts(k+1), 2), _ts(k+1), 1.0;
            A_vec_[i].block(idx_, (8*k), 1, 8) = temp_;

            // Y[idx, i] = path0[k+1, i]
            Y_(idx_, i) = _traj_setpoints((k+1), i);
            idx_ += 1;                                      // iterate

            // A[idx, 8*(k+1):8*(k+2), i]
            temp_ << std::pow(_ts(k+1), 7), std::pow(_ts(k+1), 6), std::pow(_ts(k+1), 5), std::pow(_ts(k+1), 4),
            std::pow(_ts(k+1), 3), std::pow(_ts(k+1), 2), _ts(k+1), 1.0;
            A_vec_[i].block(idx_, (8*(k+1)), 1, 8) = temp_;

            // Y[idx, i] = path0[k+1, i]
            Y_(idx_, i) = _traj_setpoints((k+1), i);
            idx_ += 1;                                      // iterate

        } // end contraint 1

        // Constraint 2: \dot{x}_k(t_k) = \dot{x}_{k+1}(t_k)
        for(int k = 0; k < m_-1; k++)
        {
            // A[idx, 8*k:8*(k+1), i]
            temp_ << 7.0*std::pow(_ts(k+1), 6), 6.0*std::pow(_ts(k+1), 5), 5.0*std::pow(_ts(k+1), 4),
                        4.0*std::pow(_ts(k+1), 3), 3.0*std::pow(_ts(k+1), 2), 2.0*_ts(k+1), 1.0, 0.0;
            A_vec_[i].block(idx_, (8*k), 1, 8) = temp_;

            // A[idx, 8*(k+1):8*(k+2), i]
            temp_ << -7.0*std::pow(_ts(k+1), 6), -6.0*std::pow(_ts(k+1), 5), -5.0*std::pow(_ts(k+1), 4),
                        -4.0*std::pow(_ts(k+1), 3), -3.0*std::pow(_ts(k+1), 2), -2.0*_ts(k+1), -1.0, 0.0;
            A_vec_[i].block(idx_, (8*(k+1)), 1, 8) = temp_;
            Y_(idx_, i) = 0.0;
            idx_ += 1;                                      // iterate

        } // end constraint 2

        // Constraint 3: \ddot{x}_k(t_k) = \ddot{x}_{k+1}(t_k)
        for(int k = 0; k < m_-1; k++)
        {
            // A[idx, 8*k:8*(k+1), i]
            temp_ << 42.0*std::pow(_ts(k+1), 5), 30.0*std::pow(_ts(k+1), 4), 20.0*std::pow(_ts(k+1), 3),
                        12.0*std::pow(_ts(k+1), 2), 6.0*_ts(k+1), 2.0, 0.0, 0.0;
            A_vec_[i].block(idx_, (8*k), 1, 8) = temp_;

            // A[idx, 8*(k+1):8*(k+2), i]
            temp_ << -42.0*std::pow(_ts(k+1), 5), -30.0*std::pow(_ts(k+1), 4), -20.0*std::pow(_ts(k+1), 3),
                        -12.0*std::pow(_ts(k+1), 2), -6.0*_ts(k+1), -2.0, 0.0, 0.0;
            A_vec_[i].block(idx_, (8*(k+1)), 1, 8) = temp_;
            Y_(idx_, i) = 0.0;
            idx_ += 1;                                      // iterate

        } // end constraint 3

        // Constraint 4: x^(3)_k(t_k) = x^(3)_{k+1}(t_k)
        for(int k = 0; k < m_-1; k++)
        {
            // A[idx, 8*k:8*(k+1), i]
            temp_ << 210.0*std::pow(_ts(k+1), 4), 120.0*std::pow(_ts(k+1), 3), 60.0*std::pow(_ts(k+1), 2),
                        24.0*_ts(k+1), 6.0, 0.0, 0.0, 0.0;
            A_vec_[i].block(idx_, (8*k), 1, 8) = temp_;

            // A[idx, 8*(k+1):8*(k+2), i]
            temp_ << -210.0*std::pow(_ts(k+1), 4), -120.0*std::pow(_ts(k+1), 3), -60.0*std::pow(_ts(k+1), 2),
                    -24.0*_ts(k+1), -6.0, 0.0, 0.0, 0.0;
            A_vec_[i].block(idx_, (8*(k+1)), 1, 8) = temp_;
            Y_(idx_, i) = 0.0;
            idx_ += 1;                                      // iterate

        } // end constraint 4

        // Constraint 5: x^(4)_k(t_k) = x^(4)_{k+1}(t_k)
        for(int k = 0; k < m_-1; k++)
        {
            // A[idx, 8*k:8*(k+1), i]
            temp_ << 840.0*std::pow(_ts(k+1), 3), 360.0*std::pow(_ts(k+1), 2), 120.0*_ts(k+1), 24.0, 0.0, 0.0,
                        0.0, 0.0;
            A_vec_[i].block(idx_, (8*k), 1, 8) = temp_;

            // A[idx, 8*(k+1):8*(k+2), i]
            temp_ << -840.0*std::pow(_ts(k+1), 3), -360.0*std::pow(_ts(k+1), 2), -120.0*_ts(k+1), -24.0, 0.0, 0.0,
                        0.0, 0.0;
            A_vec_[i].block(idx_, (8*(k+1)), 1, 8) = temp_;
            Y_(idx_, i) = 0.0;
            idx_ += 1;                                      // iterate

        } // end constraint 5

        // Constraint 6: x^(5)_k(t_k) = x^(5)_{k+1}(t_k)
        for(int k = 0; k < m_-1; k++)
        {
            // A[idx, 8*k:8*(k+1), i]
            temp_ << 2520.0*std::pow(_ts(k+1), 2), 720.0*_ts(k+1), 120.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            A_vec_[i].block(idx_, (8*k), 1, 8) = temp_;

            // A[idx, 8*(k+1):8*(k+2), i]
            temp_ << -2520.0*std::pow(_ts(k+1), 2), -720.0*_ts(k+1), -120.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            A_vec_[i].block(idx_, (8*(k+1)), 1, 8) = temp_;
            Y_(idx_, i) = 0.0;
            idx_ += 1;                                      // iterate

        } // end constraint 6

        // Constraint 7: x^(6)_k(t_k) = x^(6)_{k+1}(t_k)
        for(int k = 0; k < m_-1; k++)
        {
            // A[idx, 8*k:8*(k+1), i]
            temp_ << 5040.0*_ts(k+1), 720.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            A_vec_[i].block(idx_, (8*k), 1, 8) = temp_;

            // A[idx, 8*(k+1):8*(k+2), i]
            temp_ << -5040.0*_ts(k+1), -720.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            A_vec_[i].block(idx_, (8*(k+1)), 1, 8) = temp_;
            Y_(idx_, i) = 0.0;
            idx_ += 1;                                      // iterate

        } // end constraint 7

        // 4 of the last 8 constraints to be added: (already 8(m-1) in place...
        int k = 0;

        // from constraint 1
        temp_ << std::pow(_ts(k), 7), std::pow(_ts(k), 6), std::pow(_ts(k), 5), std::pow(_ts(k), 4),
                    std::pow(_ts(k), 3), std::pow(_ts(k), 2), _ts(k), 1.0;
        A_vec_[i].block(idx_, (8*k), 1, 8) = temp_;
        Y_(idx_, i) = _traj_setpoints(k, i);
        idx_ += 1;                                      // iterate

        // from constraint 2
        temp_ << 7.0*std::pow(_ts(k), 6), 6.0*std::pow(_ts(k), 5), 5.0*std::pow(_ts(k), 4),
                    4.0*std::pow(_ts(k), 3), 3.0*std::pow(_ts(k), 2), 2.0*_ts(k), 1.0, 0.0;
        A_vec_[i].block(idx_, (8*k), 1, 8) = temp_;
        Y_(idx_, i) = 0.0;
        idx_ += 1;                                      // iterate

        // from constraint 3
        temp_ << 42.0*std::pow(_ts(k), 5), 30.0*std::pow(_ts(k), 4), 20.0*std::pow(_ts(k), 3),
                    12.0*std::pow(_ts(k), 2), 6.0*_ts(k), 2.0, 0.0, 0.0;
        A_vec_[i].block(idx_, (8*k), 1, 8) = temp_;
        Y_(idx_, i) = 0.0;
        idx_ += 1;                                      // iterate

        //from constraint 4
        temp_ << 210.0*std::pow(_ts(k), 4), 120.0*std::pow(_ts(k), 3), 60.0*std::pow(_ts(k), 2),
                24.0*_ts(k), 6.0, 0.0, 0.0, 0.0;
        A_vec_[i].block(idx_, (8*k), 1, 8) = temp_;
        Y_(idx_, i) = 0.0;
        idx_ += 1;                                      // iterate

        // Last 4 of the last 8 constraints to be added:
        k = m_-1;
//        k = m_;
//        std::cout << "vec: " << i << std::endl;
//        std::cout << A_vec_[i] << std::endl;
//        std::cout << std::endl;
//        std::cout << A_vec_[i].size() << std::endl;
//        std::cout << std::endl
//        std::cout << _ts << std::endl;
//        std::cout << std::endl;

        // from constraint 1
        temp_ << std::pow(_ts(k+1), 7), std::pow(_ts(k+1), 6), std::pow(_ts(k+1), 5), std::pow(_ts(k+1), 4),
                    std::pow(_ts(k+1), 3), std::pow(_ts(k+1), 2), _ts(k+1), 1.0;
        A_vec_[i].block(idx_, (8*k), 1, 8) = temp_;
        Y_(idx_, i) = _traj_setpoints(k+1, i);
        idx_ += 1;                                      // iterate

        // from constraint 2
        temp_ << 7.0*std::pow(_ts(k+1), 6), 6.0*std::pow(_ts(k+1), 5), 5.0*std::pow(_ts(k+1), 4),
                    4.0*std::pow(_ts(k+1), 3), 3.0*std::pow(_ts(k+1), 2), 2.0*_ts(k+1), 1.0, 0.0;
        A_vec_[i].block(idx_, (8*k), 1, 8) = temp_;
        Y_(idx_, i) = 0.0;
        idx_ += 1;                                      // iterate

        // from constraint 3
        temp_ << 42.0*std::pow(_ts(k+1), 5), 30.0*std::pow(_ts(k+1), 4), 20.0*std::pow(_ts(k+1), 3),
                    12.0*std::pow(_ts(k+1), 2), 6.0*_ts(k+1), 2.0, 0.0, 0.0;
        A_vec_[i].block(idx_, (8*k), 1, 8) = temp_;
        Y_(idx_, i) = 0.0;
        idx_ += 1;                                      // iterate

        //from constraint 4
        temp_ << 210.0*std::pow(_ts(k+1), 4), 120.0*std::pow(_ts(k+1), 3), 60.0*std::pow(_ts(k+1), 2),
                    24.0*_ts(k+1), 6.0, 0.0, 0.0, 0.0;
        A_vec_[i].block(idx_, (8*k), 1, 8) = temp_;
        Y_(idx_, i) = 0.0;
        idx_ += 1;                                      // iterate

        // solve linear system of equations
        // https://stackoverflow.com/questions/53247078/c-eigen-for-solving-linear-systems-fast
        // https://eigen.tuxfamily.org/dox/TopicUsingIntelMKL.html
        _coef.col(i) = A_vec_[i].lu().solve(Y_.col(i));

    } // end for(i)  -- position dimensions x, y, z
} // end min_snap_optimization()

//// Open-loop control for level take-off
// theoretical take-off rotor rate is 655.0
void test_ol_takeoff()
{
    ref_motor_vel0.set_data(690.0);
    ref_motor_vel1.set_data(690.0);
    ref_motor_vel2.set_data(690.0);
    ref_motor_vel3.set_data(690.0);

} // end test_ol_takeoff()

//// Open-loop control for level-ish landing
// practical hover point is 665.0
void test_ol_land()
{
    ref_motor_vel0.set_data(645.0);
    ref_motor_vel1.set_data(645.0);
    ref_motor_vel2.set_data(645.0);
    ref_motor_vel3.set_data(645.0);

} // end test_ol_land()

//// Set rotor values before publishing
void test_cl_takeoff()
{
    ref_motor_vel0.set_data(_desired_thrust(0));
    ref_motor_vel1.set_data(_desired_thrust(1));
    ref_motor_vel2.set_data(_desired_thrust(2));
    ref_motor_vel3.set_data(_desired_thrust(3));

} // end test_cl_takeoff()

//// Initialize variables to a known state
void initialize_variables()
{
    //
    // Different controller gains for different purposes
    //

     // Soft controller -- good for recovering from large disturbances
//    _Kp_pos << 3.0, 3.0, 10.0;
//    _Kd_pos << 3.0, 3.0, 6.0;
//    _Kp_ang << 700.0, 700.0, 0.0;
//    _Kd_ang << 100000.0, 100000.0, 0.0;         // TODO: add in yaw tracking for a dedicated "front/forward"

    //  Testing/Intermediate Controller -- the in-between controller
//    _Kp_pos << 5.0, 5.0, 10.0;
//    _Kd_pos << 3.0, 3.0, 6.0;
//    _Kp_ang << 700.0, 700.0, 0.0;
//    _Kd_ang << 100000.0, 100000.0, 0.0;

    // Fast/Stiff controller -- performance tracking for aggressive maneuvers
    _Kp_pos << 8.0, 8.0, 30.0;
    _Kd_pos << 3.8, 4.1, 10.0;
    _Kp_ang << 1100.0, 1100.0, 1000.0;
    _Kd_ang << 120000.0, 120000.0, 200000.0;

    // Initial setpoints to achieve
    _desired_pos << 0.0, 0.0, 2.0;              // linear position; x, y, z (in meters)
    _desired_vel << 0.0, 0.0, 0.0;              // linear velocity; x, y, z
    _desired_acc << 0.0, 0.0, 0.0;              // linear acceleration; x, y, z
    _desired_euler_att << 0.0, 0.0, 0.0;        // roll, pitch, yaw (in radians)
    _orig_desired_euler_att = _desired_euler_att;
    _desired_pqr_att << 0.0, 0.0, 0.0;          // angular rates (this is not the same as euler rate of change)

    // trajectory setpoints -- minimum snap
//    _traj_setpoints << _desired_pos(0), _desired_pos(1), _desired_pos(2),
//                    1.0,                  0.0,                  _desired_pos(2),
//                    1.0,                  1.0,                  _desired_pos(2),
//                    0.0,                  1.0,                  _desired_pos(2),
//                    -1.0,                 1.0,                  _desired_pos(2),
//                    -1.0,                 0.0,                  _desired_pos(2);

    _traj_setpoints <<
            _desired_pos(0),   _desired_pos(1),  _desired_pos(2),
            2.0,                     1.0,                    _desired_pos(2),
            3.0,                     2.0,                    _desired_pos(2),
            2.0,                     3.0,                    _desired_pos(2),
            3.0,                     5.0,                    _desired_pos(2),
            5.0,                     5.0,                    _desired_pos(2);

    // Initialize data that will be changing through operation
    _sensor_quat << 1.0, 0.0, 0.0, 0.0;         // define upright zeroed orientation
    _quat_normalized << 1.0, 0.0, 0.0, 0.0;     // quaternions always need to be "unit length"
    _derived_euler_att << 0.0, 0.0, 0.0;        // converted from quaternion measurements in quat2euler()
    _final_att_deltas << 0.0, 0.0, 0.0;         // for logging purposes

} // end initialize_variables()

//// Derive sensor values from position and orientation measurements
void derived_sensor_values()
{
    // Base clock defined here at 1MHz
    if ((sim_state == 2) & ((sim_time - prev_sim_time) > 0.000001))                      // currently running at the micro-second scale
    {
        // TODO: sometimes I get a 0.0 evaluation for lin_vel_z between otherwise good values, not sure why
        sim_time_delta = sim_time - prev_sim_time;              // time slice used to derive velocity values

        _derived_lin_vel(0) = (_sensor_pos(0) - _prev_sensor_pos(0)) / sim_time_delta;
        _derived_lin_vel(1) = (_sensor_pos(1) - _prev_sensor_pos(1)) / sim_time_delta;
        _derived_lin_vel(2) = (_sensor_pos(2) - _prev_sensor_pos(2)) / sim_time_delta;

        Eigen::Matrix<double,1,3> euler_ = quat2euler(_sensor_quat);        // convert from quaternion to euler angles

        _derived_pqr_att = derive_ang_velocity(euler_);                     // produces angular velocity vector

        // store previous position value
        _prev_sensor_pos(0) = _sensor_pos(0);
        _prev_sensor_pos(1) = _sensor_pos(1);
        _prev_sensor_pos(2) = _sensor_pos(2);

        // store previous attitude value
        _prev_derived_euler_att(0) = euler_(0);
        _prev_derived_euler_att(1) = euler_(1);
        _prev_derived_euler_att(2) = euler_(2);

        // TODO: some of these pitch values are an order of magnitude or more off
//        std::cout << "Pitch: " << euler_(1) << std::endl;
//        std::cout << "Time slice: " << std::fixed << std::setprecision(8) << sim_time_delta << std::endl;

        prev_sim_time = sim_time;                   // should be the last thing run

    } // end if(sim_state == 2 and 1MHz clock cycle)
} // end derived_sensor_values()

//// Hover-envelope position controller

void basic_position_controller()
{
    // References (papers):
    // Trajectory Generation and Control for Precise Aggressive Maneuvers with Quadrotors
    // The GRASP Multiple Micro UAV Testbed
    //

    // produce desired acceleration vector; includes feed-forward acceleration term
    Eigen::Array3d acc_des_ = _desired_acc + (1.0*_Kd_pos.cwiseProduct(_desired_vel - _derived_lin_vel))
                                + (1.0*_Kp_pos.cwiseProduct(_desired_pos - _sensor_pos));

    // calculate desired euler attitudes for the attitude controller
    _desired_euler_att(0) = (1.0/_gravity) * ((acc_des_(0)*sin(_desired_euler_att(2))) + (acc_des_(1)*cos(_desired_euler_att(2))));
    _desired_euler_att(1) = (1.0/_gravity) * ((acc_des_(0)*-1.0*cos(_desired_euler_att(2))) + (acc_des_(1)*sin(_desired_euler_att(2))));
    _desired_euler_att(2) = _derived_euler_att(2);                    // desired yaw is forward-facing
//    _desired_euler_att(2) = _orig_desired_euler_att(2);     // yaw is a free variable

    // For testing the attitude controller; circumvents the position controller except for hover/altitude controller
//    _desired_euler_att(0) = _orig_desired_euler_att(0);
//    _desired_euler_att(1) = _orig_desired_euler_att(1);
//    _desired_euler_att(2) = _orig_desired_euler_att(2);

    _desired_tot_thrust_delta = (_mass / (8.0 * _motor_force_const * _hover_point)) * acc_des_(2);      // hover/altitude control

} // end basic_position_controller()

//// Attitude controller; uses error feedback (approximates SO(3))
// same reference papers as above
void basic_attitude_controller()
{
    // quaternion is being normalized by Gazebo
    _derived_euler_att = quat2euler(_sensor_quat);

    Eigen::Array3d att_deltas_;

    // calculate attitude deltas for mapping to motor thrusts
    att_deltas_(0) = 1.0*(_Kp_ang(0) * (_desired_euler_att(0) - _derived_euler_att(0)))
                            + 1.0*(_Kd_ang(0) * (_desired_pqr_att(0) - _derived_pqr_att(0)));
    att_deltas_(1) = 1.0*(_Kp_ang(1) * (_desired_euler_att(1) - _derived_euler_att(1)))
                            + 1.0*(_Kd_ang(1) * (_desired_pqr_att(1) - _derived_pqr_att(1)));
    att_deltas_(2) = 1.0*(_Kp_ang(2) * (_desired_euler_att(2) - _derived_euler_att(2)))
                            + 1.0*(_Kd_ang(2) * (_desired_pqr_att(2) - _derived_pqr_att(2)));

    _final_att_deltas = att_deltas_;        // for logging purposes

    // 4x1 vector to be multiplied by the thrust mapping matrix
    Eigen::Matrix<double,4,1> all_deltas_;
    all_deltas_ << (_hover_point + _desired_tot_thrust_delta), att_deltas_(0), att_deltas_(1), att_deltas_(2);

    _desired_thrust =  (_motor_mapping * all_deltas_);          // derive desired rotor rates

} // end basic_attitude_controller()

//
// HELPER FUNCTIONS
//

// References
// https://stackoverflow.com/questions/28585653/use-of-lpnorm-in-eigen
// https://eigen.tuxfamily.org/dox/TopicFunctionTakingEigenTypes.html
// https://stackoverflow.com/questions/35612831/eigenref-in-pass-by-pointer

//// Convert quaternions to a rotation matrix
// used in quat2euler()
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

//// Convert quaternions to euler angles

Eigen::Matrix<double,1,3> quat2euler(const Eigen::Ref<const Eigen::Matrix<double,1,4>>& q_)
{
    // References
    // previously listed papers
    // Nice approximation of atan2: https://www.dsprelated.com/showarticle/1052.php
    // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    // http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/
    //

    Eigen::Matrix3d rotation_;
    Eigen::Matrix<double,1,3> euler_;
    euler_ << 0.0, 0.0, 0.0;

    rotation_ = quat2rot(q_);
    euler_(0) = asin(rotation_(1,2));   // roll
    euler_(1) = atan2((-1.0*rotation_(0,2)) / cos(euler_(0)),
                                   (rotation_(2,2) / cos(euler_(0))));
    euler_(2) = atan2((-1.0*rotation_(1,0)) / cos(euler_(0)),
                      (rotation_(1,1) / cos(euler_(0))));

    // TODO: this is another way to make the conversion; there are issues here though
    // TODO: this other method does not fix the discontinuities in roll and pitch (I think it's a sensor value issue)
//    euler_(0) = atan2((2*((-q_(0)*q_(1)) + (q_(2)*q_(3)))), (1-2*(q_(1)*q_(1) + q_(2)*q_(2))));
//    euler_(1) = asin(2*(-q_(0)*q_(2) - q_(3)*q_(1)));
//    euler_(2) = atan2((2*((-q_(0)*q_(3)) + (q_(1)*q_(2)))), (1-2*(q_(2)*q_(2) + q_(3)*q_(3))));

    return(euler_);

} // end quat2euler()

//// Derives angular velocity vector from euler angles
Eigen::Matrix<double,1,3> derive_ang_velocity(const Eigen::Ref<const Eigen::Matrix<double,1,3>>& e_)
{
    // References:
    // The GRASP Multiple Micro UAV Testbed
    //

    Eigen::Matrix<double,3,3> tfm_;             // transformation matrix

    tfm_(0,0) = cos(e_(1)); tfm_(0,1) = 0.0; tfm_(0,2) = (-1.0*cos(e_(0))*sin(e_(1)));
    tfm_(1,0) = 0.0;                 tfm_(1,1) = 1.0; tfm_(1,2) = sin(e_(0));
    tfm_(2,0) = sin(e_(1)); tfm_(2,1) = 0.0; tfm_(2,2) = (cos(e_(0))*cos(e_(1)));

    _derived_pqr_att = tfm_ * (e_ - _prev_derived_euler_att).transpose();       // angular velocity vector

    return(_derived_pqr_att);

} // end derive_ang_velocity()

#pragma clang diagnostic pop