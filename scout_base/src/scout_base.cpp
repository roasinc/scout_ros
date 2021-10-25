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

#include "scout_base/scout_base.h"

ScoutBase::ScoutBase(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
  : nh_(nh)
  , nh_priv_(nh_priv)
  , port_("/dev/ttyUSB0")
  , baud_(115200)
  , robot_name_("scout")
  , control_frequency_(50.0)
  , previous_state_(false)
{
  nh_priv_.getParam("port", port_);
  nh_priv_.getParam("baud", baud_);
  nh_priv_.getParam("robot_name", robot_name_);
  nh_priv_.getParam("control_frequency", control_frequency_);

  driver_ = make_shared<ScoutDriver>(robot_name_, port_, baud_, robot_state_, motor_state_, driver_state_, light_state_,
                                     battery_state_);
  hw_ = make_shared<ScoutHardware>(joint_);
  cm_ = make_shared<controller_manager::ControllerManager>(hw_.get(), nh_);
  // diagnostics_ = make_shared<ScoutDiagnostics>(robot_, base_, motor_, light_);

  sub_front_light_cmd_ = nh_.subscribe(robot_name_ + "/front_light/command", 10, &ScoutBase::frontLightCallback, this);
  sub_rear_light_cmd_ = nh_.subscribe(robot_name_ + "/rear_light/command", 10, &ScoutBase::rearLightCallback, this);
  clear_failure_ = nh_.advertiseService(robot_name_ + "/clear_failure", &ScoutBase::clearFailure, this);
}

bool ScoutBase::init()
{
  rp_robot_.init(nh_, robot_name_ + "/robot_feedback", 1);
  rp_motor_.init(nh_, robot_name_ + "/motor_feedback", 1);
  rp_driver_.init(nh_, robot_name_ + "/driver_feedback", 1);
  rp_light_.init(nh_, robot_name_ + "/light_feedback", 1);
  rp_battery_.init(nh_, robot_name_ + "/battery_feedback", 1);
  estop_.init(nh_, "/hardware_estop", 1);

  rp_robot_.msg_.robot = robot_name_;
  for (size_t i = 0; i < joint_.size(); i++)
    rp_motor_.msg_.name[i] = joint_[i];
  estop_.msg_.data = false;

  if (!driver_->connect())
    return false;

  if (!driver_->init())
    return false;

  return true;
}

void ScoutBase::frontLightCallback(const scout_msgs::LightCommand& msg)
{
  LightCommand cmd;
  cmd.mode = msg.mode;
  cmd.custom_brightness = msg.custom_brightness;
  driver_->sendLightCommand("front", cmd);
}

void ScoutBase::rearLightCallback(const scout_msgs::LightCommand& msg)
{
  LightCommand cmd;
  cmd.mode = msg.mode;
  cmd.custom_brightness = msg.custom_brightness;
  driver_->sendLightCommand("rear", cmd);
}

bool ScoutBase::clearFailure(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
  driver_->clearFailure();
  resp.success = true;
  return true;
}

void ScoutBase::publishRobotState()
{
  if (rp_robot_.trylock())
  {
    rp_robot_.msg_.header.stamp = ros::Time::now();
    rp_robot_.msg_.state = robot_state_.state;
    rp_robot_.msg_.control_mode = robot_state_.control_mode;
    rp_robot_.msg_.fault_state.battery_under_voltage_failure = robot_state_.fault_state.battery_under_voltage_failure;
    rp_robot_.msg_.fault_state.battery_under_voltage_alarm = robot_state_.fault_state.battery_under_voltage_alarm;
    rp_robot_.msg_.fault_state.loss_remote_control = robot_state_.fault_state.loss_remote_control;

    rp_robot_.unlockAndPublish();
  }
}

void ScoutBase::publishMotorState()
{
  if (rp_motor_.trylock())
  {
    rp_motor_.msg_.header.stamp = ros::Time::now();

    for (size_t i = 0; i < 4; i++)
    {
      rp_motor_.msg_.position[i] = motor_state_.position[i];
      rp_motor_.msg_.velocity[i] = motor_state_.velocity[i];
      rp_motor_.msg_.current[i] = motor_state_.current[i];
      rp_motor_.msg_.temperature[i] = motor_state_.temperature[i];
    }

    rp_motor_.unlockAndPublish();
  }
}

void ScoutBase::publishDriverState()
{
  if (rp_driver_.trylock())
  {
    rp_driver_.msg_.header.stamp = ros::Time::now();

    for (size_t i = 0; i < 4; i++)
    {
      rp_driver_.msg_.driver_voltage[i] = driver_state_.driver_voltage[i];
      rp_driver_.msg_.driver_temperature[i] = driver_state_.driver_temperature[i];
      rp_driver_.msg_.communication_failure[i] = driver_state_.communication_failure[i];
      rp_driver_.msg_.low_supply_voltage[i] = driver_state_.low_supply_voltage[i];
      rp_driver_.msg_.motor_over_temperature[i] = driver_state_.motor_over_temperature[i];
      rp_driver_.msg_.driver_over_current[i] = driver_state_.driver_over_current[i];
      rp_driver_.msg_.driver_over_temperature[i] = driver_state_.driver_over_temperature[i];
      rp_driver_.msg_.sensor_normal_state[i] = driver_state_.sensor_normal_state[i];
      rp_driver_.msg_.driver_error_state[i] = driver_state_.driver_error_state[i];
      rp_driver_.msg_.driver_enable_state[i] = driver_state_.driver_enable_state[i];
    }

    rp_driver_.unlockAndPublish();
  }
}

void ScoutBase::publishLightState()
{
  if (rp_light_.trylock())
  {
    rp_light_.msg_.header.stamp = ros::Time::now();
    rp_light_.msg_.control_enable = light_state_.control_enable;
    rp_light_.msg_.front_light.mode = light_state_.front_light.mode;
    rp_light_.msg_.front_light.brightness = light_state_.front_light.brightness;
    rp_light_.msg_.rear_light.mode = light_state_.rear_light.mode;
    rp_light_.msg_.rear_light.brightness = light_state_.rear_light.brightness;

    rp_light_.unlockAndPublish();
  }
}

void ScoutBase::publishBatteryState()
{
  if (rp_battery_.trylock())
  {
    rp_battery_.msg_.header.stamp = ros::Time::now();
    rp_battery_.msg_.SOC = battery_state_.SOC;
    rp_battery_.msg_.SOH = battery_state_.SOH;
    rp_battery_.msg_.voltage = battery_state_.voltage;
    rp_battery_.msg_.current = battery_state_.current;
    rp_battery_.msg_.temperature = battery_state_.temperature;

    rp_battery_.unlockAndPublish();
  }
}

void ScoutBase::publishEstopState()
{
  if (driver_->estop_state_ != previous_state_)
  {
    if (estop_.trylock())
    {
      estop_.msg_.data = driver_->estop_state_;
      estop_.unlockAndPublish();
    }
    previous_state_ = estop_.msg_.data;
  }
}

void ScoutBase::publishLoop()
{
  while (ros::ok())
  {
    publishRobotState();
    publishMotorState();
    publishDriverState();
    publishLightState();
    publishBatteryState();
    publishEstopState();
    // diagnostics_->updateDiagnosticsMessage();

    ros::Rate(control_frequency_).sleep();
  }
}

void ScoutBase::controlLoop()
{
  chrono::steady_clock::time_point last_time = chrono::steady_clock::now();

  while (ros::ok())
  {
    chrono::steady_clock::time_point current_time = chrono::steady_clock::now();
    chrono::duration<double> elapsed_time = current_time - last_time;
    ros::Duration elapsed(elapsed_time.count());
    last_time = current_time;

    hw_->receiveMotorState(motor_state_);
    cm_->update(ros::Time::now(), elapsed);
    hw_->writeCommand(motor_cmd_);
    driver_->sendMotorCommand(motor_cmd_);

    ros::Rate(control_frequency_).sleep();
  }
}

int main(int argc, char** argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "scout_base_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  auto scout = make_shared<ScoutBase>(nh, nh_priv);

  if (scout->init())
  {
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Create thread for controller manager loop
    thread contol([&scout]() -> void { scout->controlLoop(); });

    // Create thread for publishing the feedback data
    thread publish([&scout]() -> void { scout->publishLoop(); });

    while (ros::ok())
      scout->driver_->read();

    spinner.stop();
  }

  return EXIT_FAILURE;
}