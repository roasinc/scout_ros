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

#ifndef SCOUT_LIB__RESOURCE_H_
#define SCOUT_LIB__RESOURCE_H_

#include <string>
#include <vector>
#include <array>

using namespace std;

struct MotorCommand
{
  array<double, 4> command = { 0.0, 0.0, 0.0, 0.0 };
};

struct LightCommand
{
  uint8_t mode;               // 0: NC, 1: NO, 2: BL, 3: CUSTOM
  uint8_t custom_brightness;  // Only for CUSTOM mode
};

struct FaultState
{
  bool battery_under_voltage_failure = false;
  bool battery_under_voltage_alarm = false;
  bool loss_remote_control = false;
};

struct RobotState
{
  bool normal_state = false;
  string control_mode = "NONE";  // IDLE, CAN, SERIAL, REMOTE, NONE
  double battery_voltage = 0.0;
  FaultState fault_state;
};

struct MotorState
{
  // The actual position by the encoder value (rad)
  array<double, 4> position = { 0.0, 0.0, 0.0, 0.0 };

  // The actual velocity measured by the encoder as the actual RPM value (rad/s)
  array<double, 4> velocity = { 0.0, 0.0, 0.0, 0.0 };

  // The current flowing through the motor (A)
  array<double, 4> current = { 0.0, 0.0, 0.0, 0.0 };

  // The actual temperature of motor (C)
  array<int16_t, 4> temperature = { 0, 0, 0, 0 };
};

struct DriverState
{
  array<double, 4> driver_voltage = { 0.0, 0.0, 0.0, 0.0 };
  array<int16_t, 4> driver_temperature = { 0, 0, 0, 0 };
  array<bool, 4> communication_failure = { false, false, false, false };
  array<bool, 4> low_supply_voltage = { false, false, false, false };
  array<bool, 4> motor_over_temperature = { false, false, false, false };
  array<bool, 4> driver_over_current = { false, false, false, false };
  array<bool, 4> driver_over_temperature = { false, false, false, false };
  array<bool, 4> sensor_normal_state = { false, false, false, false };
  array<bool, 4> driver_error_state = { false, false, false, false };
  array<bool, 4> driver_enable_state = { false, false, false, false };
};

struct Light
{
  string mode = "NONE";  // The current mode (NC, NO, BL, CUSTOM)
  uint8_t brightness;    // The current brightness of light (0 - 100)
};

struct LightState
{
  bool control_enable = false;  // Lighting control enable flag

  Light front_light;
  Light rear_light;
};

struct BatteryState
{
  uint8_t SOC = 0;
  uint8_t SOH = 0;
  double voltage = 0.0;
  double current = 0.0;
  double temperature = 0.0;
};

#endif  // SCOUT_LIB__RESOURCE_H_