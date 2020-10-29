// Hardware in the Loop interface to control Quadcopter in the Gazebo simulation environment
//
// By: Patrick Ledzian, Julian Blanco
// Date: 06 October 2020
//

#include "include/HIL_quad_control.hpp"
#include <boost/thread.hpp>
// #include <boost/thread/condition>.


#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"


 int globalfiledescriptor;
 Eigen::Matrix<double,1,3> _euler;
 char str1[200] = "<";
 boost::mutex threadRunning;
  boost::mutex writeData;
   boost::mutex readData;
     char buf [100];
    Eigen::Matrix<double,1,4> motor_values ;

    FILE* rwfile;
//// Subscriber callback functions
// Provides measured motor velocity


 void wait(int seconds)
{
  boost::this_thread::sleep_for(boost::chrono::seconds{seconds});
}


 void waitMillis(int seconds)
{
  boost::this_thread::sleep_for(boost::chrono::milliseconds{seconds});
}

void writeThread()
{
    int i =0;
    Eigen::Matrix<double,1,4> _write_quat((Eigen::Matrix<double,1,4>() << 1.0, 0.0, 0.0, 0.0).finished());
    Eigen::Matrix<double,1,3> _write_pos;
    Eigen::Matrix<double,1,3> _write_euler;
  while(!threadRunning.try_lock())
  {
        writeData.lock();
        _write_quat = _sensor_quat;
        _write_pos = _sensor_pos;
        writeData.unlock();

        _write_euler = quat2euler(_write_quat);

    //<$OA008,10,20,30,2,3,4>

        int leng = snprintf(str1,sizeof(str1),"<$OA008,%f,%f,%f,%f,%f,%f>",_write_euler(0),_write_euler(1),_write_euler(2),_write_pos(0),_write_pos(1),_write_pos(2));

        write (globalfiledescriptor, str1, leng);
        fsync(globalfiledescriptor);
        // std::cout<<str1<<std::endl;
        waitMillis(20);

  }
  threadRunning.unlock();
}

void readThread()
{

    std::stringstream ss;
    std::string substr;

  while(!threadRunning.try_lock())
  {
        fgets (buf, sizeof(buf),rwfile);
        ss.str(buf);
        // std::cout  <<ss.str() ;
        readData.lock(); 
        for( int i=0;ss.good()&&i<4;i++ )
        {
            
            getline( ss, substr, ',' );
            // std::cout<<substr;
            motor_values[i]= std::stoi(substr);
        }
        readData.unlock();
        ss.clear();
  }
    threadRunning.unlock();
}


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



int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        if (tcgetattr (fd, &tty) != 0)
        {
                // error_message ("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                // error_message ("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

void set_blocking (int fd, int should_block){
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        // if (tcgetattr (fd, &tty) != 0)
        // {
        //         // error_message ("error %d from tggetattr", errno);
        //         return;
        // }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        // if (tcsetattr (fd, TCSANOW, &tty) != 0)
                // error_message ("error %d setting term attributes", errno);
}


// TODO: not sure if "iris" is the same index every run, need to run a for loop to find it and its index
// TODO: put in write lock here and read lock in derived_sensor_values()

//// Subscriber callback function
// pulls from Gazebo: sim time (seconds), linear position from origin (meters), and orientation (quaternion)
void local_poses_cb(ConstLocalPosesStampedPtr &local_pose)
{
   
    sim_time = local_pose->time().sec() + (local_pose->time().nsec() * 10E-10);
    writeData.lock();
    _sensor_pos(0) = local_pose->pose(0).position().x();
    _sensor_pos(1) = local_pose->pose(0).position().y();
    _sensor_pos(2) = local_pose->pose(0).position().z();

    _sensor_quat(0) = local_pose->pose(0).orientation().w();
    _sensor_quat(1) = local_pose->pose(0).orientation().x();
    _sensor_quat(2) = local_pose->pose(0).orientation().y();
    _sensor_quat(3) = local_pose->pose(0).orientation().z();
   writeData.unlock();
   
    

} // end local_poses_cb()

//// Main loop
// First portion sets up the messaging with Gazebo and logging file handle
// Second portion is the while(1) loop that runs for duration of quadcopter manuevers
int main(int _argc, char **_argv)
{
     motor_values[0]=0;
      motor_values[1]=0;
       motor_values[2]=0;
        motor_values[3]=0;
    //set up HIL
    const char* portname = "/dev/ttyACM0";

    globalfiledescriptor = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (globalfiledescriptor < 0)
    {
            // error_message ("error %d opening %s: %s", errno, portname, strerror (errno));
            std::cout << "error" << std::endl;
            return -1;
    }

    set_interface_attribs (globalfiledescriptor, B2000000, 0);  // set speed to 115,200 bps, 8n1 (no parity)
    set_blocking (globalfiledescriptor, 0);                // set no blocking
    rwfile = fdopen(globalfiledescriptor,"rw");

    
    
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

    threadRunning.lock();
    boost::thread readT{readThread};
    boost::thread writeT{writeThread};
    // readT.join();
    // writeT.join();

    // open file handle for storing test data for later analysis
    std::ofstream testdata;
    testdata.open("test_data.txt");
    u_int64_t ctr = 0;                  // timing counter, good for 2^64 bits (essentially infinite)




    
    while(1)
    {

        readData.lock();
        ref_motor_vel0.set_data(motor_values[0]);
        ref_motor_vel1.set_data(motor_values[1]);
        ref_motor_vel2.set_data(motor_values[2]);
        ref_motor_vel3.set_data(motor_values[3]);
        readData.unlock();
        pub0->Publish(ref_motor_vel0);
        pub1->Publish(ref_motor_vel1);
        pub2->Publish(ref_motor_vel2);
        pub3->Publish(ref_motor_vel3);
        




       // std::cout << "sensor motor 0: " << motor_values[0] << std::endl;
       // std::cout << "sensor motor 1: " << motor_values[1] << std::endl;
       // // std::cout << "sensor motor 2: " << sensor_motor_vel2 << std::endl;
       // // std::cout << "sensor motor 3: " << sensor_motor_vel3 << std::endl;
       // std::cout << std::endl;
       waitMillis(5);
    } // end while(1)

    threadRunning.unlock();
    return(0);

} // end main()

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

        prev_sim_time = sim_time;                   // should be the last thing run

    } // end if(sim_state == 2 and 1MHz clock cycle)
} // end derived_sensor_values()


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
// references are the papers in the controller comments
// Nice approximation of atan2: https://www.dsprelated.com/showarticle/1052.php
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

    return(euler_);

} // end quat2euler()

//// Derives angular velocity vector from euler angles
Eigen::Matrix<double,1,3> derive_ang_velocity(const Eigen::Ref<const Eigen::Matrix<double,1,3>>& e_)
{
    Eigen::Matrix<double,3,3> tfm_;             // transformation matrix

    tfm_(0,0) = cos(e_(1)); tfm_(0,1) = 0.0; tfm_(0,2) = (-1.0*cos(e_(0))*sin(e_(1)));
    tfm_(1,0) = 0.0; tfm_(1,1) = 1.0; tfm_(1,2) = sin(e_(0));
    tfm_(2,0) = sin(e_(1)); tfm_(2,1) = 0.0; tfm_(2,2) = (cos(e_(0))*cos(e_(1)));

    _derived_pqr_att = tfm_ * (e_ - _prev_derived_euler_att).transpose();       // angular velocity vector

    return(_derived_pqr_att);

} // end derive_ang_velocity()

#pragma clang diagnostic pop