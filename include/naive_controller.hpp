// Testing the ability to set rotor velocity via publish-subscribe
// 
// By: Patrick Ledzian
// Date: 02 September 2020

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>

namespace gazebo {

class NaiveController {
public:
	NaiveController() {}

	virtual ~NaiveController();

protected:
	virtual void AttControl();

private:
	transport::PublisherPtr motor_velocity_pub_;


}; // end class NaiveController



}