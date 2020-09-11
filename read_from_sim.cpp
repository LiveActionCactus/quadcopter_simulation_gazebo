/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <gazebo/transport/transport.hh>

#include <gazebo/msgs/msgs.hh>

#include <gazebo/gazebo_client.hh>

#include "SITLGps.pb.h"

#include <iostream>

#include <errno.h>

#include <fcntl.h>

#include <string.h>

#include <termios.h>

#include <unistd.h>


/////////////////////////////////////////////////
// Function is called everytime a message is received.
void cb(ConstWorldStatisticsPtr & _msg) {
  // Dump the message contents to stdout.
  // std::cout << _msg -> DebugString();
}

/////////////////////////////////////////////////
// Function is called everytime a message is received.
void cb2(ConstIMUPtr & _msg) {
  // Dump the message contents to stdout.
  // std::cout << _msg->DebugString();
}
// Function is called everytime a message is received.
void cb3(const boost::shared_ptr <
  const sensor_msgs::msgs::SITLGps > & _msg) {
  // Dump the message contents to stdout.
  // std::cout << _msg -> DebugString();
}

/////////////////////////////////////////////////
int main(int _argc, char ** _argv) {

  // Load gazebo
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node -> Init();

  // Listen to Gazebo world_stats topic
  // gazebo::transport::SubscriberPtr sub = node->Subscribe("~/world_stats", cb);
  // // gazebo::transport::SubscriberPtr sub2 = node->Subscribe("~/iris/iris/imu_link/imu_sensor/imu", cb2);

  gazebo::transport::SubscriberPtr sub2 = node -> Subscribe("~/imu/link/imu/imu", cb2);
  gazebo::transport::SubscriberPtr sub3 = node -> Subscribe("~/3DR_gps_mag/link/ublox-neo-7", cb3);
  // gps_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + gps_sub_topic_, &GazeboMavlinkInterface::GpsCallback, this);
  // Busy wait loop...replace with your own code as needed.
  while (true)
    gazebo::common::Time::MSleep(10);

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}