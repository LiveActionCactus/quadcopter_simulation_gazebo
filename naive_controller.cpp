// Testing the ability to set rotor velocity via publish-subscribe
// 
// By: Patrick Ledzian
// Date: 02 September 2020


#include "naive_controller.hpp"

namespace gazebo {
	NaiveController::~NaiveController() {
		std::cout << "object made" << std::endl
	}

	void NaiveController::AttControl() {
		;
	}
}