/** Contains the addresses of registers of the Turtlebot3 conveyor.
 *
 * The control table should match the configuration of the DYNAMIXEL::Slave
 * implemented in the example sketches of the "Turtlebot3 ROS2" Arduino
 * library, in the file "conveyor.cpp", enum ControlTableItemAddr.
 */
// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Darby Lim

#pragma once

#include <stdlib.h>

#include <cstdint>  // For uint8_t, uint16_t.

namespace robotis
{
namespace turtlebot3
{
constexpr uint8_t EEPROM = 1;
constexpr uint8_t RAM = 2;

constexpr uint8_t READ = 1;
constexpr uint8_t READ_WRITE = 3;

typedef struct
{
  uint16_t addr;
  uint8_t memory;
  uint16_t length;
  uint8_t rw;
} ControlItem;

/* The addresses must correspond to the definition of enum ControlTableItemAddr
 * in conveyor.cpp of the sketch running on the OpenCR.
 */
// In vim, you can use `'<,'>s/ADDR_\(\w*\)\s*=\s*\(\d\+\),.*/ControlItem \L\1\E = {\2, RAM, ?, READ?};` to format from the sketch.
typedef struct
{
  ControlItem model_number = {0, EEPROM, 2, READ};
  ControlItem model_information = {2, EEPROM, 4, READ};
  ControlItem firmware_version = {6, EEPROM, 1, READ};
  ControlItem id = {7, EEPROM, 1, READ};
  ControlItem baud_rate = {8, EEPROM, 1, READ};

  ControlItem millis = {10, RAM, 4, READ};
  ControlItem micros = {14, RAM, 4, READ};

  ControlItem device_status = {18, RAM, 1, READ};
  ControlItem heartbeat = {19, RAM, 1, READ_WRITE};

  ControlItem external_led_1 = {20, RAM, 1, READ_WRITE};
  ControlItem external_led_2 = {21, RAM, 1, READ_WRITE};
  ControlItem external_led_3 = {22, RAM, 1, READ_WRITE};
  ControlItem external_led_4 = {23, RAM, 1, READ_WRITE};

  ControlItem button_1 = {26, RAM, 1, READ};
  ControlItem button_2 = {27, RAM, 1, READ};

  ControlItem bumper_1 = {28, RAM, 1, READ};
  ControlItem bumper_2 = {29, RAM, 1, READ};

  ControlItem illumination = {30, RAM, 4, READ};
  ControlItem ir = {34, RAM, 4, READ};
  ControlItem sonar = {38, RAM, 4, READ};

  ControlItem battery_voltage = {42, RAM, 4, READ};
  ControlItem battery_percentage = {46, RAM, 4, READ};

  ControlItem sound = {50, RAM, 1, READ_WRITE};

  ControlItem imu_re_calibration = {59, RAM, 1, READ_WRITE};

  ControlItem imu_angular_velocity_x = {60, RAM, 4, READ};
  ControlItem imu_angular_velocity_y = {64, RAM, 4, READ};
  ControlItem imu_angular_velocity_z = {68, RAM, 4, READ};
  ControlItem imu_linear_acceleration_x = {72, RAM, 4, READ};
  ControlItem imu_linear_acceleration_y = {76, RAM, 4, READ};
  ControlItem imu_linear_acceleration_z = {80, RAM, 4, READ};
  ControlItem imu_magnetic_x = {84, RAM, 4, READ};
  ControlItem imu_magnetic_y = {88, RAM, 4, READ};
  ControlItem imu_magnetic_z = {92, RAM, 4, READ};
  ControlItem imu_orientation_w = {96, RAM, 4, READ};
  ControlItem imu_orientation_x = {100, RAM, 4, READ};
  ControlItem imu_orientation_y = {104, RAM, 4, READ};
  ControlItem imu_orientation_z = {108, RAM, 4, READ};

  ControlItem present_joint_current_0 = {120, RAM, 4, READ};
  ControlItem present_joint_current_1 = {124, RAM, 4, READ};
  ControlItem present_joint_current_2 = {128, RAM, 4, READ};
  ControlItem present_joint_current_3 = {132, RAM, 4, READ};
  ControlItem present_joint_position_0 = {136, RAM, 4, READ};
  ControlItem present_joint_position_1 = {140, RAM, 4, READ};
  ControlItem present_joint_position_2 = {144, RAM, 4, READ};
  ControlItem present_joint_position_3 = {148, RAM, 4, READ};
  ControlItem present_joint_velocity_0 = {152, RAM, 4, READ};
  ControlItem present_joint_velocity_1 = {156, RAM, 4, READ};
  ControlItem present_joint_velocity_2 = {160, RAM, 4, READ};
  ControlItem present_joint_velocity_3 = {164, RAM, 4, READ};
  ControlItem present_wheel_current_0 = {168, RAM, 4, READ};
  ControlItem present_wheel_current_1 = {172, RAM, 4, READ};
  ControlItem present_wheel_current_2 = {176, RAM, 4, READ};
  ControlItem present_wheel_current_3 = {180, RAM, 4, READ};
  ControlItem present_wheel_position_0 = {184, RAM, 4, READ};
  ControlItem present_wheel_position_1 = {188, RAM, 4, READ};
  ControlItem present_wheel_position_2 = {192, RAM, 4, READ};
  ControlItem present_wheel_position_3 = {196, RAM, 4, READ};
  ControlItem present_wheel_velocity_0 = {200, RAM, 4, READ};
  ControlItem present_wheel_velocity_1 = {204, RAM, 4, READ};
  ControlItem present_wheel_velocity_2 = {208, RAM, 4, READ};
  ControlItem present_wheel_velocity_3 = {212, RAM, 4, READ};

  /* Used command velocity. */
  ControlItem cmd_vel_source = {220, RAM, 1, READ};
  ControlItem used_cmd_vel_linear_x = {221, RAM, 4, READ};
  ControlItem used_cmd_vel_linear_y = {225, RAM, 4, READ};
  ControlItem used_cmd_vel_angular_z = {229, RAM, 4, READ};

  /* Position from odom, velocity from odom. */
  ControlItem odom_x = {234, RAM, 4, READ};
  ControlItem odom_y = {238, RAM, 4, READ};
  ControlItem odom_yaw = {242, RAM, 4, READ};
  ControlItem reset_odom = {246, RAM, 1, READ_WRITE};

  ControlItem motor_torque_enable = {249, RAM, 1, READ_WRITE};

  ControlItem cmd_velocity_linear_x = {250, RAM, 4, READ_WRITE};
  ControlItem cmd_velocity_linear_y = {254, RAM, 4, READ_WRITE};
  ControlItem cmd_velocity_linear_z = {258, RAM, 4, READ_WRITE};
  ControlItem cmd_velocity_angular_x = {262, RAM, 4, READ_WRITE};
  ControlItem cmd_velocity_angular_y = {266, RAM, 4, READ_WRITE};
  ControlItem cmd_velocity_angular_z = {270, RAM, 4, READ_WRITE};

  ControlItem profile_acceleration_0 = {274, RAM, 4, READ_WRITE};
  ControlItem profile_acceleration_1 = {278, RAM, 4, READ_WRITE};
  ControlItem profile_acceleration_2 = {282, RAM, 4, READ_WRITE};
  ControlItem profile_acceleration_3 = {284, RAM, 4, READ_WRITE};
} ControlTable;

const ControlTable g_extern_control_table;
}  // namespace turtlebot3
}  // namespace robotis
