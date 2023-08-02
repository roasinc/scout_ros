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

#ifndef SCOUT_LIB__SCOUT_HAREWARE_H_
#define SCOUT_LIB__SCOUT_HAREWARE_H_

#include <string>
#include <vector>
#include <chrono>
#include <cmath>
#include <thread>

#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/robot_hw.h"

#include "scout_lib/resource.h"

using namespace std;

class ScoutHardware : public hardware_interface::RobotHW
{
public:
  ScoutHardware(vector<string> joint);

  virtual ~ScoutHardware() = default;

  /**
   * \brief Register joint handle and joint state handle
   */
  void registerHardwareInterface();

  /**
   * \brief Write command message to hardware interface
   * \param cmd Command to motor controller
   */
  void writeCommand(MotorCommand& cmd);

  /**
   * \brief Motor state callback
   * \param state  Motor state data
   */
  void receiveMotorState(const MotorState& state);

private:
  /// Hardware interface
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;

  chrono::steady_clock::time_point last_time_;

  /// Data of joint handles
  vector<double> cmd_, pos_, vel_, eff_;

  /// Joint name
  vector<string> joint_;
};

#endif  // SCOUT_LIB__SCOUT_HAREWARE_H_