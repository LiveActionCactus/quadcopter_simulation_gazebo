#include "include/set_rotor_vel.hpp"
//#include "include/rotor_plugin.hpp"

/////////////////////////////////////////////////
///

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
typedef const boost::shared_ptr<const std_msgs::msgs::Float> MotorSpeedPtr;         // points to protobuf custom message "Float"
//typedef const boost::shared_ptr<const gazebo::msgs::PoseStamped> ConstPosesStampedPtr ;
//typedef const boost::shared_ptr<const gazebo::msgs::PosesStampedPtr> ConstPosesStampedPtr;

typedef const boost::shared_ptr<const gazebo::msgs::LocalPosesStamped> ConstLocalPosesStampedPtr;

gazebo::transport::NodePtr node_handle;

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

// Actions
std::string takeoff ("takeoff");

//
// Callback functions providing measured motor velocity
//
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

// TODO: not sure if "iris" is the same index every run, need to run a for loop to find it and its index
void local_poses_cb(ConstLocalPosesStampedPtr &local_pose)
{
    sensor_pos_x = local_pose->pose(0).position().x();
    sensor_pos_y = local_pose->pose(0).position().y();
    sensor_pos_z = local_pose->pose(0).position().z();

    sensor_rot_w = local_pose->pose(0).orientation().w();
    sensor_rot_w = local_pose->pose(0).orientation().x();
    sensor_rot_w = local_pose->pose(0).orientation().y();
    sensor_rot_w = local_pose->pose(0).orientation().z();

    std::cout << local_pose->pose(0).name().data() << std::endl;
    std::cout << local_pose->pose(0).position().x() << std::endl;
    std::cout << local_pose->pose(0).position().y() << std::endl;
    std::cout << local_pose->pose(0).position().z() << std::endl;

    std::cout << local_pose->pose(0).orientation().w() << std::endl;
    std::cout << local_pose->pose(0).orientation().x() << std::endl;
    std::cout << local_pose->pose(0).orientation().y() << std::endl;
    std::cout << local_pose->pose(0).orientation().z() << std::endl;
    std::cout << std::endl;

}


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

    // TODO: implement a controller where the rotor velocities can be incorperated (I read this paper...)
//    // Subscribe to measured rotor velocities
//    gazebo::transport::SubscriberPtr sub0 = node_handle->Subscribe("/gazebo/default/iris/motor_speed/0", rotor0_cb);
//    gazebo::transport::SubscriberPtr sub1 = node_handle->Subscribe("/gazebo/default/iris/motor_speed/1", rotor1_cb);
//    gazebo::transport::SubscriberPtr sub2 = node_handle->Subscribe("/gazebo/default/iris/motor_speed/2", rotor2_cb);
//    gazebo::transport::SubscriberPtr sub3 = node_handle->Subscribe("/gazebo/default/iris/motor_speed/3", rotor3_cb);

    // Subscribe to measured position and orientation values
    gazebo::transport::SubscriberPtr sub5 = node_handle->Subscribe("~/pose/info", local_poses_cb);

    while(1)
    {
        if(takeoff == _argv[1]){
            test_ol_takeoff();
        } else {
            test_ol_land();
        }

        pub0->Publish(ref_motor_vel0);
        pub1->Publish(ref_motor_vel1);
        pub2->Publish(ref_motor_vel2);
        pub3->Publish(ref_motor_vel3);

    } // end while(1)

} // end main()

void test_ol_takeoff()
{
    ref_motor_vel0.set_data(670.0);
    ref_motor_vel1.set_data(670.0);
    ref_motor_vel2.set_data(670.0);
    ref_motor_vel3.set_data(670.0);
}

void test_ol_land()
{
    ref_motor_vel0.set_data(645.0);
    ref_motor_vel1.set_data(645.0);
    ref_motor_vel2.set_data(645.0);
    ref_motor_vel3.set_data(645.0);
}
#pragma clang diagnostic pop