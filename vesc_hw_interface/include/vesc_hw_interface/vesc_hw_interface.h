/*********************************************************************
 * Copyright (c) 2019, SoftBank Corp.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************/

#ifndef VESC_HW_INTERFACE_VESC_HW_INTERFACE_H_
#define VESC_HW_INTERFACE_VESC_HW_INTERFACE_H_

#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <controller_manager/controller_manager.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <urdf_parser/urdf_parser.h>
#include <pluginlib/class_list_macros.hpp>

#include "vesc_driver/vesc_interface.h"
#include "vesc_driver/vesc_packet.h"
#include "vesc_driver/vesc_packet_factory.h"
#include "vesc_hw_interface/vesc_servo_controller.h"

namespace vesc_hw_interface
{
using vesc_driver::VescInterface;
using vesc_driver::VescPacket;
using vesc_driver::VescPacketValues;

class VescHwInterface : public hardware_interface::RobotHW
{
public:
  VescHwInterface();
  ~VescHwInterface();

  bool init(ros::NodeHandle&, ros::NodeHandle&);
  void read();
  void read(const ros::Time&, const ros::Duration&);
  void write();
  void write(const ros::Time&, const ros::Duration&);
  ros::Time getTime() const;
  ros::Duration getPeriod() const;

private:
  VescInterface vesc_interface_;
  VescServoController servo_controller_;

  std::string joint_name_, command_mode_;

  double command_;
  double position_, velocity_, effort_;  // joint states

  int num_motor_pole_pairs_;          // the number of motor pole pairs
  double gear_ratio_, torque_const_;  // physical params.

  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface joint_position_interface_;
  hardware_interface::VelocityJointInterface joint_velocity_interface_;
  hardware_interface::EffortJointInterface joint_effort_interface_;

  joint_limits_interface::JointLimits joint_limits_;
  joint_limits_interface::PositionJointSaturationInterface limit_position_interface_;
  joint_limits_interface::VelocityJointSaturationInterface limit_velocity_interface_;
  joint_limits_interface::EffortJointSaturationInterface limit_effort_interface_;

  void packetCallback(const std::shared_ptr<VescPacket const>&);
  void errorCallback(const std::string&);
};

}  // namespace vesc_hw_interface

#endif  // VESC_HW_INTERFACE_VESC_HW_INTERFACE_H_
