/*
Software License Agreement (BSD License)

Authors : Brighten Lee <shlee@roas.co.kr>

Copyright (c) 2021, ROAS Inc.
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

#ifndef SCOUT_LIB__SCOUT_CAN_BRIDGE_H_
#define SCOUT_LIB__SCOUT_CAN_BRIDGE_H_

#include <string>
#include <array>
#include <vector>

#include "ros/ros.h"
#include "can_msgs/Frame.h"
#include "socketcan_interface/socketcan.h"
#include "socketcan_interface/threading.h"
#include "socketcan_interface/xmlrpc_settings.h"
#include "socketcan_interface/filter.h"
#include "socketcan_interface/string.h"

#include "scout_lib/resource.h"

using namespace std;

class CANBridge
{
public:
  CANBridge(ros::NodeHandle* nh, ros::NodeHandle* nh_param, can::DriverInterfaceSharedPtr driver,
            RobotState& robot_state, MotorState& motor_state, DriverState& driver_state, LightState& light_state,
            BatteryState& battery_state, double wheel_radius, double wheel_separation_);

  void setup();

  void setup(const can::FilteredFrameListener::FilterVector& filters);

  void setup(XmlRpc::XmlRpcValue filters);

  void setup(ros::NodeHandle nh);

  void convertSocketCANToMessage(const can::Frame& f, can_msgs::Frame& m);

  void convertMessageToSocketCAN(const can_msgs::Frame& m, can::Frame& f);

  /// Feedback data
  RobotState& robot_state_;
  MotorState& motor_state_;
  DriverState& driver_state_;
  LightState& light_state_;
  BatteryState& battery_state_;

  // Estop state
  bool estop_state_;

private:
  void frameCallback(const can::Frame& f);

  void stateCallback(const can::State& s);

  /**
   * \brief Parse the robot state message
   * \param data Robot state message converted to binary number
   */
  void robotState(uint8_t* data);

  /**
   * \brief Parse the motor state message
   * \param index Index of motor
   * \param data Motor state converted to binary number
   */
  void motorState(size_t index, uint8_t* data);

  /**
   * \brief Parse the driver state message
   * \param index Index of driver
   * \param data Driver state converted to binary number
   */
  void driverState(size_t index, uint8_t* data);

  /**
   * \brief Parse the light state message
   * \param data Light state message converted to binary number
   */
  void lightState(uint8_t* data);

  /**
   * \brief Parse the velocity message
   * \param data Velocity message converted to binary number
   */
  void velocity(uint8_t* data);

  /**
   * \brief Parse the position message
   * \param data Position message converted to binary number
   */
  void position(uint8_t* data);

  /**
   * \brief Parse the battery state message
   * \param data Battery state message converted to binary number
   */
  void batteryState(uint8_t* data);

  ros::Publisher can_topic_;

  can::DriverInterfaceSharedPtr driver_;
  can::FrameListenerConstSharedPtr frame_listener_;
  can::StateListenerConstSharedPtr state_listener_;

  /// Robot parametres
  double wheel_radius_, wheel_separation_;
};

#endif  // SCOUT_LIB__SCOUT_CAN_BRIDGE_H_