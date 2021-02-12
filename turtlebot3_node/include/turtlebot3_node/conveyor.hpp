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

#include <turtlebot3_node/conveyor_control_table.hpp>  // robotis::turtlebot3::g_extern_control_table.
#include <turtlebot3_node/conveyor_devices/devices.hpp>
#include <turtlebot3_node/conveyor_devices/motor_power.hpp>
#include <turtlebot3_node/conveyor_devices/reset.hpp>
#include <turtlebot3_node/conveyor_devices/sound.hpp>
#include <turtlebot3_node/conveyor_sensors/battery_state.hpp>
#include <turtlebot3_node/conveyor_sensors/imu.hpp>
#include <turtlebot3_node/conveyor_sensors/joint_state.hpp>
#include <turtlebot3_node/conveyor_sensors/sensor_state.hpp>
#include <turtlebot3_node/conveyor_sensors/sensors.hpp>
#include <turtlebot3_node/dynamixel_sdk_wrapper.hpp>
#include <turtlebot3_node/odometry.hpp>

#include <geometry_msgs/msg/twist.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <turtlebot3_msgs/msg/sensor_state.hpp>

#include <array>
#include <chrono>
#include <list>
#include <map>
#include <memory>  // std::shared_ptr.
#include <mutex>
#include <queue>
#include <string>

namespace robotis
{
namespace turtlebot3
{

using robotis::turtlebot3::g_extern_control_table;

class Conveyor : public rclcpp::Node
{
  public:

    struct Motors
    {
      float profile_acceleration_constant;
      float profile_acceleration;
    };

    explicit Conveyor(const std::string & usb_port);
    virtual ~Conveyor() {}

    Motors * getMotors();

  private:

    void initDynamixelSdkWrapper(const std::string & usb_port);
    void checkDeviceStatus();

    void addSensors();
    void addDevices();
    void addMotors();

    void run();

    void initPublishTimer(const std::chrono::milliseconds timeout);
    void initHeartBeatTimer(const std::chrono::milliseconds timeout);

    void initCmdVelSubscriber();
    void initParamClient();

    Motors motors_;

    std::shared_ptr<DynamixelSDKWrapper> dxl_sdk_wrapper_;

    std::list<std::shared_ptr<sensors::Sensors>> sensors_;
    std::map<std::string, devices::Devices *> devices_;

    std::unique_ptr<Odometry> odom_;

    rclcpp::Node::SharedPtr node_handle_;

    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    rclcpp::AsyncParametersClient::SharedPtr priv_parameters_client_;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;
};

}  // namespace turtlebot3
}  // namespace robotis
