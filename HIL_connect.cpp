
#include "include/set_rotor_vel.hpp"
//#include "include/rotor_plugin.hpp"
#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <sstream>

/////////////////////////////////////////////////
///

int
set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        if (tcgetattr (fd, &tty) != 0)
        {
                std::cout << ("error %d from tcgetattr", errno);
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
                std::cout << ("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

void
set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                std::cout << ("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                std::cout << ("error %d setting term attributes", errno);
}

typedef const boost::shared_ptr<const std_msgs::msgs::Float> MotorSpeedPtr;         // points to protobuf custom message "Float"
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

int motor_0_speed = 100;
int motor_1_speed = 100;
int motor_2_speed = 100;
int motor_3_speed = 100;

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


  
    const char* portname = "/dev/ttyUSB0";

    int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);

    if (fd < 0)
    {
            std::cout << ("error %d opening %s: %s", errno, portname, strerror (errno));
            return 1;
    }

    set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
    set_blocking (fd, 0);                // set no blocking
     FILE* rwfile = fdopen(fd,"rw");
    char buf [100];
    std::vector<int> vect;
    int val;
    std::stringstream ss;
    fgets (buf, sizeof(buf),rwfile); 
    fgets (buf, sizeof(buf),rwfile); 
    fgets (buf, sizeof(buf),rwfile); 
    fgets (buf, sizeof(buf),rwfile); 
    fgets (buf, sizeof(buf),rwfile); 


    while(1)
    {
        fgets (buf, sizeof(buf),rwfile);
        ss.str(buf);
        std::cout  <<ss.str() ; 
        while( ss.good() )
        {
            std::string substr;
            getline( ss, substr, ',' );
            // std::cout<<substr;
            vect.push_back( std::stoi(substr) );
        }

        ref_motor_vel0.set_data(vect[0]);
        ref_motor_vel1.set_data(vect[1]);
        ref_motor_vel2.set_data(vect[2]);
        ref_motor_vel3.set_data(vect[3]);

        pub0->Publish(ref_motor_vel0);
        pub1->Publish(ref_motor_vel1);
        pub2->Publish(ref_motor_vel2);
        pub3->Publish(ref_motor_vel3);
        vect.clear();
        ss.clear();
//        std::cout << "sensor motor 0: " << sensor_motor_vel0 << std::endl;
//        std::cout << "sensor motor 1: " << sensor_motor_vel1 << std::endl;
//        std::cout << "sensor motor 2: " << sensor_motor_vel2 << std::endl;
//        std::cout << "sensor motor 3: " << sensor_motor_vel3 << std::endl;
//        std::cout << std::endl;

    } // end while(1)

    return(0);

} // end main()



