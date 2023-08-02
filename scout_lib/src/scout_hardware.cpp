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

#include "scout_lib/scout_hardware.h"

ScoutHardware::ScoutHardware(vector<string> joint) : joint_()
{
  for (size_t i = 0; i < joint.size(); i++)
    joint_.push_back(joint[i]);

  cmd_.assign(4, 0.0);
  pos_.assign(4, 0.0);
  vel_.assign(4, 0.0);
  eff_.assign(4, 0.0);

  last_time_ = std::chrono::steady_clock::now();
  registerHardwareInterface();
}

void ScoutHardware::registerHardwareInterface()
{
  for (int i = 0; i < 4; i++)
  {
    // Connect and register the joint state interface
    hardware_interface::JointStateHandle joint_state_handle(joint_[i], &pos_[i], &vel_[i], &eff_[i]);
    joint_state_interface_.registerHandle(joint_state_handle);

    // Connect and register the joint velocity interface
    hardware_interface::JointHandle joint_handle(joint_state_handle, &cmd_[i]);
    velocity_joint_interface_.registerHandle(joint_handle);
  }
  registerInterface(&joint_state_interface_);
  registerInterface(&velocity_joint_interface_);
}

void ScoutHardware::writeCommand(MotorCommand& cmd)
{
  for (size_t i = 0; i < 4; i++)
    cmd.command[i] = cmd_[i];
}

void ScoutHardware::receiveMotorState(const MotorState& state)
{
  chrono::steady_clock::time_point cur_time = chrono::steady_clock::now();
  chrono::duration<double> elapsed_time = cur_time - last_time_;
  const double dt = elapsed_time.count();
  last_time_ = cur_time;

  for (size_t i = 0; i < joint_.size(); i++)
  {
    pos_[i] += state.velocity[i] * dt;
    vel_[i] = state.velocity[i];
    eff_[i] = state.current[i];
  }
}