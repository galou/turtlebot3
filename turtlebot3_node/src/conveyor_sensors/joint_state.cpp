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

#include <turtlebot3_node/conveyor_sensors/joint_state.hpp>

#include <turtlebot3_node/conveyor_control_table.hpp>  // robotis::turtlebot3::g_extern_control_table.

#include <array>
#include <cstdint>  // int32_t, size_t.
#include <memory>
#include <string>
#include <utility>

using robotis::turtlebot3::g_extern_control_table;
using robotis::turtlebot3::sensors::JointState;

JointState::JointState(
  std::shared_ptr<rclcpp::Node> & nh,
  const std::string & topic_name,
  const std::string & frame_id)
: Sensors(nh, frame_id)
{
  pub_ = nh->create_publisher<sensor_msgs::msg::JointState>(topic_name, this->qos_);

  RCLCPP_INFO(nh_->get_logger(), "Succeeded to create joint state publisher");
}

void JointState::publish(
  const rclcpp::Time & now,
  std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper)
{
  sensor_msgs::msg::JointState msg;

  /* With int32_t, we can drive the wheel motors for approx 4.3 million km so
   * we don't need to consider overflow. */

  std::array<int32_t, JOINT_NUM> positions{
    dxl_sdk_wrapper->get_data_from_device<int32_t>(
      g_extern_control_table.present_joint_position_0.addr,
      g_extern_control_table.present_joint_position_0.length),
    dxl_sdk_wrapper->get_data_from_device<int32_t>(
      g_extern_control_table.present_joint_position_1.addr,
      g_extern_control_table.present_joint_position_1.length),
    dxl_sdk_wrapper->get_data_from_device<int32_t>(
      g_extern_control_table.present_joint_position_2.addr,
      g_extern_control_table.present_joint_position_2.length),
    dxl_sdk_wrapper->get_data_from_device<int32_t>(
      g_extern_control_table.present_joint_position_3.addr,
      g_extern_control_table.present_joint_position_3.length),
    dxl_sdk_wrapper->get_data_from_device<int32_t>(
      g_extern_control_table.present_wheel_position_0.addr,
      g_extern_control_table.present_wheel_position_0.length),
    dxl_sdk_wrapper->get_data_from_device<int32_t>(
      g_extern_control_table.present_wheel_position_1.addr,
      g_extern_control_table.present_wheel_position_1.length),
    dxl_sdk_wrapper->get_data_from_device<int32_t>(
      g_extern_control_table.present_wheel_position_2.addr,
      g_extern_control_table.present_wheel_position_2.length),
    dxl_sdk_wrapper->get_data_from_device<int32_t>(
      g_extern_control_table.present_wheel_position_3.addr,
      g_extern_control_table.present_wheel_position_3.length),
  };

  std::array<int32_t, JOINT_NUM> velocities{
    dxl_sdk_wrapper->get_data_from_device<int32_t>(
      g_extern_control_table.present_joint_velocity_0.addr,
      g_extern_control_table.present_joint_velocity_0.length),
    dxl_sdk_wrapper->get_data_from_device<int32_t>(
      g_extern_control_table.present_joint_velocity_1.addr,
      g_extern_control_table.present_joint_velocity_1.length),
    dxl_sdk_wrapper->get_data_from_device<int32_t>(
      g_extern_control_table.present_joint_velocity_2.addr,
      g_extern_control_table.present_joint_velocity_2.length),
    dxl_sdk_wrapper->get_data_from_device<int32_t>(
      g_extern_control_table.present_joint_velocity_3.addr,
      g_extern_control_table.present_joint_velocity_3.length),
    dxl_sdk_wrapper->get_data_from_device<int32_t>(
      g_extern_control_table.present_wheel_velocity_0.addr,
      g_extern_control_table.present_wheel_velocity_0.length),
    dxl_sdk_wrapper->get_data_from_device<int32_t>(
      g_extern_control_table.present_wheel_velocity_1.addr,
      g_extern_control_table.present_wheel_velocity_1.length),
    dxl_sdk_wrapper->get_data_from_device<int32_t>(
      g_extern_control_table.present_wheel_velocity_2.addr,
      g_extern_control_table.present_wheel_velocity_2.length),
    dxl_sdk_wrapper->get_data_from_device<int32_t>(
      g_extern_control_table.present_wheel_velocity_3.addr,
      g_extern_control_table.present_wheel_velocity_3.length),
  };

  std::array<int32_t, JOINT_NUM> currents{ 
    dxl_sdk_wrapper->get_data_from_device<int32_t>(
      g_extern_control_table.present_joint_current_0.addr,
      g_extern_control_table.present_joint_current_0.length),
    dxl_sdk_wrapper->get_data_from_device<int32_t>(
      g_extern_control_table.present_joint_current_1.addr,
      g_extern_control_table.present_joint_current_1.length),
    dxl_sdk_wrapper->get_data_from_device<int32_t>(
      g_extern_control_table.present_joint_current_2.addr,
      g_extern_control_table.present_joint_current_2.length),
    dxl_sdk_wrapper->get_data_from_device<int32_t>(
      g_extern_control_table.present_joint_current_3.addr,
      g_extern_control_table.present_joint_current_3.length),
    dxl_sdk_wrapper->get_data_from_device<int32_t>(
      g_extern_control_table.present_wheel_current_0.addr,
      g_extern_control_table.present_wheel_current_0.length),
    dxl_sdk_wrapper->get_data_from_device<int32_t>(
      g_extern_control_table.present_wheel_current_1.addr,
      g_extern_control_table.present_wheel_current_1.length),
    dxl_sdk_wrapper->get_data_from_device<int32_t>(
      g_extern_control_table.present_wheel_current_2.addr,
      g_extern_control_table.present_wheel_current_2.length),
    dxl_sdk_wrapper->get_data_from_device<int32_t>(
      g_extern_control_table.present_wheel_current_3.addr,
      g_extern_control_table.present_wheel_current_3.length),
  };

  msg.header.frame_id = frame_id_;
  msg.header.stamp = now;

  msg.name.push_back("joint_0");
  msg.name.push_back("joint_1");
  msg.name.push_back("joint_2");
  msg.name.push_back("joint_3");
  msg.name.push_back("wheel_0");
  msg.name.push_back("wheel_1");
  msg.name.push_back("wheel_2");
  msg.name.push_back("wheel_3");

  msg.position.reserve(JOINT_NUM);
  msg.velocity.reserve(JOINT_NUM);
  msg.effort.reserve(JOINT_NUM);
  for (size_t i = 0; i < (JOINT_NUM / 2); ++i)
  {
    msg.position.push_back(JOINT_ROBOT_TO_RAD * positions[i]);
    msg.position.push_back(JOINT_ROBOT_TO_RAD * velocities[i]);
    msg.effort.push_back(currents[i]);
  }
  for (size_t i = (JOINT_NUM / 2); i < JOINT_NUM; ++i)
  {
    msg.position.push_back(WHEEL_ROBOT_TO_RAD * positions[i]);
    msg.velocity.push_back(WHEEL_ROBOT_TO_RAD * velocities[i]);
    msg.effort.push_back(currents[i]);
  }

  pub_->publish(std::move(msg));
}
