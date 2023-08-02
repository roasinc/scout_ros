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

#include <bitset>

#include "scout_lib/scout_can_bridge.h"

namespace can
{
template <>
can::FrameFilterSharedPtr tofilter(const XmlRpc::XmlRpcValue& ct)
{
  XmlRpc::XmlRpcValue t(ct);
  try  // try to read as integer
  {
    uint32_t id = static_cast<int>(t);
    return tofilter(id);
  }
  catch (...)  // else read as string
  {
    return tofilter(static_cast<string>(t));
  }
}
}  // namespace can

CANBridge::CANBridge(ros::NodeHandle* nh, ros::NodeHandle* nh_param, can::DriverInterfaceSharedPtr driver,
                     RobotState& robot_state, MotorState& motor_state, DriverState& driver_state,
                     LightState& light_state, BatteryState& battery_state, double wheel_radius,
                     double wheel_separation_)
  : driver_(driver)
  , robot_state_(robot_state)
  , motor_state_(motor_state)
  , driver_state_(driver_state)
  , light_state_(light_state)
  , battery_state_(battery_state)
  , estop_state_(false)
  , wheel_radius_(wheel_radius)
  , wheel_separation_(wheel_separation_)
{
  can_topic_ = nh->advertise<can_msgs::Frame>("messages", 1);
}

void CANBridge::setup()
{
  frame_listener_ = driver_->createMsgListenerM(this, &CANBridge::frameCallback);
  state_listener_ = driver_->createStateListenerM(this, &CANBridge::stateCallback);
}

void CANBridge::setup(const can::FilteredFrameListener::FilterVector& filters)
{
  frame_listener_.reset(
      new can::FilteredFrameListener(driver_, bind(&CANBridge::frameCallback, this, placeholders::_1), filters));

  state_listener_ = driver_->createStateListenerM(this, &CANBridge::stateCallback);
}

void CANBridge::setup(XmlRpc::XmlRpcValue filters)
{
  setup(can::tofilters(filters));
}

void CANBridge::setup(ros::NodeHandle nh)
{
  XmlRpc::XmlRpcValue filters;
  if (nh.getParam("can_ids", filters))
    return setup(filters);
  return setup();
}

void CANBridge::convertSocketCANToMessage(const can::Frame& f, can_msgs::Frame& m)
{
  m.id = f.id;
  m.dlc = f.dlc;
  m.is_error = f.is_error;
  m.is_rtr = f.is_rtr;
  m.is_extended = f.is_extended;

  for (int i = 0; i < 8; i++)
  {
    m.data[i] = f.data[i];
  }
}

void CANBridge::convertMessageToSocketCAN(const can_msgs::Frame& m, can::Frame& f)
{
  f.id = m.id;
  f.dlc = m.dlc;
  f.is_error = m.is_error;
  f.is_rtr = m.is_rtr;
  f.is_extended = m.is_extended;

  for (int i = 0; i < 8; i++)
  {
    f.data[i] = m.data[i];
  }
};

void CANBridge::frameCallback(const can::Frame& f)
{
  if (!f.isValid())
  {
    ROS_ERROR("Invalid frame from SocketCAN: id: %#04x, length: %d, is_extended: %d, is_error: %d, is_rtr: %d", f.id,
              f.dlc, f.is_extended, f.is_error, f.is_rtr);
    return;
  }
  else
  {
    if (f.is_error)
    {
      ROS_WARN("Received frame is error: %s", can::tostring(f, true).c_str());
    }
  }

  can_msgs::Frame msg;
  convertSocketCANToMessage(f, msg);

  msg.header.frame_id = "";  // empty frame is the de-facto standard for no frame.
  msg.header.stamp = ros::Time::now();

  can_topic_.publish(msg);

  uint8_t data[8];
  for (size_t i = 0; i < 8; i++)
    data[i] = f.data[i];

  switch (f.id)
  {
    case 0x211:
      robotState(data);
      break;
    case 0x251:  // front right motor
      motorState(2, data);
      break;
    case 0x252:  // rear right motor
      motorState(3, data);
      break;
    case 0x253:  // rear left motor
      motorState(1, data);
      break;
    case 0x254:  // front left motor
      motorState(0, data);
      break;
    case 0x261:  // front right motor
      driverState(2, data);
      break;
    case 0x262:  // rear right motor
      driverState(3, data);
      break;
    case 0x263:  // rear left motor
      driverState(1, data);
      break;
    case 0x264:  // front left motor
      driverState(0, data);
      break;
    case 0x231:  // light state
      lightState(data);
      break;
    case 0x221:  // velocity
      velocity(data);
      break;
    case 0x311:  // position
      position(data);
      break;
    case 0x361:  // battery
      batteryState(data);
      break;
    default:
      break;
  }
}

void CANBridge::stateCallback(const can::State& s)
{
  string err;
  driver_->translateError(s.internal_error, err);

  if (!s.internal_error)
    ROS_INFO("State: %s, asio: %s", err.c_str(), s.error_code.message().c_str());
  else
    ROS_ERROR("Error: %s, asio: %s", err.c_str(), s.error_code.message().c_str());
}

void CANBridge::robotState(uint8_t* data)
{
  if (data[0] == 0x00)
    robot_state_.normal_state = true;
  else
    robot_state_.normal_state = false;

  if (data[0] == 0x01)
    estop_state_ = true;
  else
    estop_state_ = false;

  // TODO: check the unknown bug
  if (data[1] == 0x00)
    robot_state_.control_mode = "IDLE";
  else if (data[1] == 0x01)
    robot_state_.control_mode = "CAN";
  else if (data[1] == 0x02)
    robot_state_.control_mode = "SERIAL";
  else if (data[1] == 0x03)
    robot_state_.control_mode = "REMOTE";
  else
    robot_state_.control_mode = "NONE";

  robot_state_.battery_voltage =
      static_cast<int16_t>((static_cast<uint16_t>(data[2]) << 8) | static_cast<uint16_t>(data[3])) / 10.0;

  robot_state_.fault_state.battery_under_voltage_failure = bitset<8>(data[5])[0];
  robot_state_.fault_state.battery_under_voltage_alarm = bitset<8>(data[5])[1];
  robot_state_.fault_state.loss_remote_control = bitset<8>(data[5])[2];

  driver_state_.communication_failure[0] = bitset<8>(data[5])[6];
  driver_state_.communication_failure[1] = bitset<8>(data[5])[5];
  driver_state_.communication_failure[2] = bitset<8>(data[5])[3];
  driver_state_.communication_failure[3] = bitset<8>(data[5])[4];
}

void CANBridge::motorState(size_t index, uint8_t* data)
{
  if (index < 4)
    motor_state_.current[index] = ((static_cast<uint16_t>(data[2]) << 8) | static_cast<uint16_t>(data[3])) / 10.0;
}

void CANBridge::driverState(size_t index, uint8_t* data)
{
  if (index < 4)
  {
    motor_state_.temperature[index] = static_cast<int16_t>(data[4]);
    driver_state_.driver_voltage[index] =
        ((static_cast<uint16_t>(data[0]) << 8) | static_cast<uint16_t>(data[1])) / 10.0;
    driver_state_.driver_temperature[index] =
        static_cast<int16_t>((static_cast<uint16_t>(data[2]) << 8) | static_cast<uint16_t>(data[3]));
    driver_state_.low_supply_voltage[index] = bitset<8>(data[5])[0];
    driver_state_.motor_over_temperature[index] = bitset<8>(data[5])[1];
    driver_state_.driver_over_current[index] = bitset<8>(data[5])[2];
    driver_state_.driver_over_temperature[index] = bitset<8>(data[5])[3];
    driver_state_.sensor_normal_state[index] = !bitset<8>(data[5])[4];
    driver_state_.driver_error_state[index] = bitset<8>(data[5])[5];
    driver_state_.driver_enable_state[index] = !bitset<8>(data[5])[6];
  }
}

void CANBridge::lightState(uint8_t* data)
{
  light_state_.control_enable = data[0];

  switch (data[1])
  {
    case 0x00:
      light_state_.front_light.mode = "NC";
      break;
    case 0x01:
      light_state_.front_light.mode = "NO";
      break;
    case 0x02:
      light_state_.front_light.mode = "BL";
      break;
    case 0x03:
      light_state_.front_light.mode = "CUSTOM";
      break;
    default:
      light_state_.front_light.mode = "NONE";
      break;
  }
  light_state_.front_light.brightness = data[2];

  switch (data[3])
  {
    case 0x00:
      light_state_.rear_light.mode = "NC";
      break;
    case 0x01:
      light_state_.rear_light.mode = "NO";
      break;
    case 0x02:
      light_state_.rear_light.mode = "BL";
      break;
    case 0x03:
      light_state_.rear_light.mode = "CUSTOM";
      break;
    default:
      light_state_.rear_light.mode = "NONE";
      break;
  }
  light_state_.rear_light.brightness = data[4];
}

void CANBridge::velocity(uint8_t* data)
{
  double linear = static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 8) | static_cast<uint16_t>(data[1])) / 1000.0;
  double angular =
      static_cast<int16_t>((static_cast<uint16_t>(data[2]) << 8) | static_cast<uint16_t>(data[3])) / 1000.0;

  double left = (linear - (angular * wheel_separation_ / 2.0)) / wheel_radius_;
  double right = (linear + (angular * wheel_separation_ / 2.0)) / wheel_radius_;

  motor_state_.velocity[0] = left;
  motor_state_.velocity[1] = right;
  motor_state_.velocity[2] = left;
  motor_state_.velocity[3] = right;
}

void CANBridge::position(uint8_t* data)
{
  // scout position
  int32_t left = static_cast<int32_t>((static_cast<uint32_t>(data[0]) << 24) | (static_cast<uint32_t>(data[1]) << 16) |
                                      (static_cast<uint32_t>(data[2]) << 8) | static_cast<uint32_t>(data[3]));
  int32_t right = static_cast<int32_t>((static_cast<uint32_t>(data[4]) << 24) | (static_cast<uint32_t>(data[5]) << 16) |
                                       (static_cast<uint32_t>(data[6]) << 8) | static_cast<uint32_t>(data[7]));

  motor_state_.position[0] = left / 1000.0 / wheel_radius_;
  motor_state_.position[1] = right / 1000.0 / wheel_radius_;
  motor_state_.position[2] = left / 1000.0 / wheel_radius_;
  motor_state_.position[3] = right / 1000.0 / wheel_radius_;
}

void CANBridge::batteryState(uint8_t* data)
{
  battery_state_.SOC = static_cast<uint8_t>(data[0]);
  battery_state_.SOH = static_cast<uint8_t>(data[1]);
  battery_state_.voltage =
      static_cast<int16_t>((static_cast<uint16_t>(data[2]) << 8) | static_cast<uint16_t>(data[3])) / 10.0;
  battery_state_.current =
      static_cast<int16_t>((static_cast<uint16_t>(data[4]) << 8) | static_cast<uint16_t>(data[5])) / 10.0;
  battery_state_.temperature =
      static_cast<int16_t>((static_cast<uint16_t>(data[6]) << 8) | static_cast<uint16_t>(data[7])) / 10.0;
}