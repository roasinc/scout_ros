/*
Software License Agreement (BSD License)

Authors : Brighten Lee <shlee@roas.co.kr>

Copyright (c) 2020, ROAS Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef SCOUT_BASE__SCOUT_BASE_H_
#define SCOUT_BASE__SCOUT_BASE_H_

#include "ros/ros.h"
#include "controller_manager/controller_manager.h"
#include "realtime_tools/realtime_publisher.h"

#include "scout_lib/scout_controller.h"
#include "scout_lib/scout_hardware.h"

#include "std_msgs/Bool.h"
#include "scout_msgs/RobotState.h"
#include "scout_msgs/MotorState.h"
#include "scout_msgs/DriverState.h"
#include "scout_msgs/LightState.h"
#include "scout_msgs/LightCommand.h"
#include "scout_msgs/BatteryState.h"
#include "std_srvs/Trigger.h"

using namespace std;

class ScoutBase
{
public:
  ScoutBase(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);

  virtual ~ScoutBase() = default;

  /**
   * \brief Initialize
   */
  bool init();

  /**
   * \brief Front light command callback
   * \param msg Command message
   */
  void frontLightCallback(const scout_msgs::LightCommand& msg);

  /**
   * \brief Rear light command callback
   * \param msg Command message
   */
  void rearLightCallback(const scout_msgs::LightCommand& msg);

  /**
   * \brief Clear the failure message
   */
  bool clearFailure(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);

  /**
   * \brief Loop for publishing
   */
  void publishLoop();

  /**
   * \brief Loop for controller manager
   */
  void controlLoop();

  /// Communication system with robot
  shared_ptr<ScoutController> robot_;

  /// Hardware interface for robot
  shared_ptr<ScoutHardware> hw_;

  /// Controller manager for the infrastructure to interact with controllers
  shared_ptr<controller_manager::ControllerManager> cm_;

private:
  /// ROS parameters
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  ros::Subscriber sub_front_light_cmd_, sub_rear_light_cmd_;

  realtime_tools::RealtimePublisher<scout_msgs::RobotState> rp_robot_state_;
  realtime_tools::RealtimePublisher<scout_msgs::MotorState> rp_motor_state_;
  realtime_tools::RealtimePublisher<scout_msgs::DriverState> rp_driver_state_;
  realtime_tools::RealtimePublisher<scout_msgs::LightState> rp_light_state_;
  realtime_tools::RealtimePublisher<scout_msgs::BatteryState> rp_battery_state_;
  realtime_tools::RealtimePublisher<std_msgs::Bool> estop_state_;

  ros::ServiceServer clear_failure_;

  /// Robot name
  string robot_name_;

  /// Joint name
  vector<string> joint_ = { "front_left_wheel_joint", "rear_left_wheel_joint", "front_right_wheel_joint",
                            "rear_right_wheel_joint" };

  /// Control frequency
  double control_frequency_;

  /// Motor command
  MotorCommand motor_cmd_;

  /// Feedback related
  RobotState robot_state_;
  MotorState motor_state_;
  DriverState driver_state_;
  LightState light_state_;
  BatteryState battery_state_;

  /// Previous state of Estop
  bool previous_state_;
};

#endif  // SCOUT_BASE__SCOUT_BASE_H_