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

#include <sensor_msgs/msg/joint_state.hpp>

#include <cstdint>  // For uint8_t.
#include <memory>
#include <string>

#include "turtlebot3_node/conveyor_sensors/sensors.hpp"

namespace robotis
{
namespace turtlebot3
{
namespace sensors
{

constexpr uint8_t JOINT_NUM{8};

/* The wheel radius must correspond to the definition in the firmware running
 * on the OpenCR. In m.*/
constexpr double WHEEL_RADIUS_M{0.033};

/* The joint positions and velocities are given in rad and rad/s by the robot. */
constexpr double JOINT_ROBOT_TO_RAD{1.0 / 1000.0};

/* The wheel positions and velocities are given in mm and mm/s by the robot. */
constexpr double WHEEL_ROBOT_TO_RAD{1.0 / WHEEL_RADIUS_M / 1000.0};

class JointState : public Sensors
{
public:
  explicit JointState(
    std::shared_ptr<rclcpp::Node> & nh,
    const std::string & topic_name = "joint_states",
    const std::string & frame_id = "base_link");

  void publish(
    const rclcpp::Time & now,
    std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper) override;

private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
};
}  // namespace sensors
}  // namespace turtlebot3
}  // namespace robotis
