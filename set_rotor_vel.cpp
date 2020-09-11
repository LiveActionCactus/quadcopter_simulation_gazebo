#include "include/set_rotor_vel.hpp"
//#include "include/rotor_plugin.hpp"

/////////////////////////////////////////////////
///

typedef const boost::shared_ptr<const std_msgs::msgs::Float> MotorSpeedPtr;         // points to protobuf custom message "Float"
//typedef const boost::shared_ptr<const gazebo::msgs::PoseStamped> ConstPoseStampedPtr ;
typedef const boost::shared_ptr<const gazebo::msgs::PosesStamped>  ConstPosesStampedPtr;


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

// TODO: did I need the base plugin to make this work?? The x,y values seem wrong and the z value is not reading in
void local_pose_cb(ConstPosesStampedPtr &local_pose)
{
//    std::cout << local_pose->pose().position().x() << std::endl;        // okay
//    std::cout << local_pose->pose().position().y() << std::endl;        // okay
//    std::cout << local_pose->pose().position().z() << std::endl;        // no value...

    std::cout << local_pose->time().sec() << std::endl;
    std::cout << local_pose->pose().Get(0).position().x() << std::endl;
    std::cout << local_pose->pose().Get(0).position().y() << std::endl;
    std::cout << local_pose->pose().Get(0).position().z() << std::endl;


//    std::cout << local_pose->position().x() << std::endl;
//    std::cout << local_pose->position().y() << std::endl;
//    std::cout << local_pose->position().z() << std::endl;

//    std::cout << local_pose->pose_size() << std::endl;
//    std::cout << local_pose->pose(0).name() << std::endl;
//    std::cout << local_pose->pose(1).name() << std::endl;
//    std::cout << local_pose->pose(2).name() << std::endl;
//    std::cout << local_pose->pose(3).name() << std::endl;
//
//    std::cout << local_pose->pose(2).position().x() << std::endl;
//    std::cout << local_pose->pose(2).position().y() << std::endl;
//    std::cout << local_pose->pose(2).position().z() << std::endl;

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

    // Subscribe to measured rotor velocities
    gazebo::transport::SubscriberPtr sub0 = node_handle->Subscribe("/gazebo/default/iris/motor_speed/0", rotor0_cb);
    gazebo::transport::SubscriberPtr sub1 = node_handle->Subscribe("/gazebo/default/iris/motor_speed/1", rotor1_cb);
    gazebo::transport::SubscriberPtr sub2 = node_handle->Subscribe("/gazebo/default/iris/motor_speed/2", rotor2_cb);
    gazebo::transport::SubscriberPtr sub3 = node_handle->Subscribe("/gazebo/default/iris/motor_speed/3", rotor3_cb);

    // Subscribe to measured position and orientation values
    gazebo::transport::SubscriberPtr sub4 = node_handle->Subscribe("/gazebo/default/pose/info", local_pose_cb);

    while(1)
    {
        ref_motor_vel0.set_data(200.0);
        ref_motor_vel1.set_data(200.0);
        ref_motor_vel2.set_data(200.0);
        ref_motor_vel3.set_data(200.0);

        pub0->Publish(ref_motor_vel0);
        pub1->Publish(ref_motor_vel1);
        pub2->Publish(ref_motor_vel2);
        pub3->Publish(ref_motor_vel3);

//        std::cout << "sensor motor 0: " << sensor_motor_vel0 << std::endl;
//        std::cout << "sensor motor 1: " << sensor_motor_vel1 << std::endl;
//        std::cout << "sensor motor 2: " << sensor_motor_vel2 << std::endl;
//        std::cout << "sensor motor 3: " << sensor_motor_vel3 << std::endl;
//        std::cout << std::endl;

    } // end while(1)

    return(0);

} // end main()

