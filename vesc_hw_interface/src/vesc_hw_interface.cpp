/*********************************************************************
 *
 * Copyright (c) 2019, SoftBank corp.
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
 *
 *********************************************************************/

#include "vesc_hw_interface/vesc_hw_interface.h"

namespace vesc_hw_interface
{
VescHwInterface::VescHwInterface()
  : vesc_interface_(std::string(), boost::bind(&VescHwInterface::packetCallback, this, _1),
                    boost::bind(&VescHwInterface::errorCallback, this, _1))
{
}

VescHwInterface::~VescHwInterface()
{
}

bool VescHwInterface::init(ros::NodeHandle& nh_root, ros::NodeHandle& nh)
{
  // reads a port name to open
  std::string port;
  if (!nh.getParam("vesc_hw_interface/port", port))
  {
    ROS_FATAL("VESC communication port parameter required.");
    ros::shutdown();
  }

  // attempts to open the serial port
  try
  {
    vesc_interface_.connect(port);
  }
  catch (serial::SerialException exception)
  {
    ROS_FATAL("Failed to connect to the VESC, %s.", exception.what());
    // ros::shutdown();
    return false;
  }

  // initializes the joint name
  nh.param<std::string>("vesc_hw_interface/joint_name", joint_name_, "joint_vesc");

  // loads joint limits
  std::string robot_description_name, robot_description;
  nh.param<std::string>("vesc_hw_interface/robot_description_name", robot_description_name, "/robot_description");

  // parses the urdf
  if (nh.getParam(robot_description_name, robot_description))
  {
    const urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDF(robot_description);
    const urdf::JointConstSharedPtr urdf_joint = urdf->getJoint(joint_name_);

    if (getJointLimits(urdf_joint, joint_limits_))
    {
      ROS_INFO("Joint limits are loaded");
    }
  }

  // initializes commands and states
  command_ = 0.0;
  position_ = 0.0;
  velocity_ = 0.0;
  effort_ = 0.0;

  // reads system parameters
  nh.param<double>("vesc_hw_interface/gear_ratio", gear_ratio_, 1.0);
  nh.param<double>("vesc_hw_interface/torque_const", torque_const_, 1.0);

  // reads driving mode setting
  // - assigns an empty string if param. is not found
  nh.param<std::string>("vesc_hw_interface/command_mode", command_mode_, "");
  ROS_INFO("mode: %s", command_mode_.data());

  // registers a state handle and its interface
  hardware_interface::JointStateHandle state_handle(joint_name_, &position_, &velocity_, &effort_);
  joint_state_interface_.registerHandle(state_handle);
  registerInterface(&joint_state_interface_);

  // registers specified command handle and its interface
  if (command_mode_ == "position")
  {
    hardware_interface::JointHandle position_handle(joint_state_interface_.getHandle(joint_name_), &command_);
    joint_position_interface_.registerHandle(position_handle);
    registerInterface(&joint_position_interface_);

    joint_limits_interface::PositionJointSaturationHandle limit_handle(position_handle, joint_limits_);
    limit_position_interface_.registerHandle(limit_handle);

    // initializes the servo controller
    servo_controller_.init(nh, &vesc_interface_, 1.0 / getPeriod().toSec());
  }
  else if (command_mode_ == "velocity")
  {
    hardware_interface::JointHandle velocity_handle(joint_state_interface_.getHandle(joint_name_), &command_);
    joint_velocity_interface_.registerHandle(velocity_handle);
    registerInterface(&joint_velocity_interface_);

    joint_limits_interface::VelocityJointSaturationHandle limit_handle(velocity_handle, joint_limits_);
    limit_velocity_interface_.registerHandle(limit_handle);
  }
  else if (command_mode_ == "effort" || command_mode_ == "effort_duty")
  {
    hardware_interface::JointHandle effort_handle(joint_state_interface_.getHandle(joint_name_), &command_);
    joint_effort_interface_.registerHandle(effort_handle);
    registerInterface(&joint_effort_interface_);

    joint_limits_interface::EffortJointSaturationHandle limit_handle(effort_handle, joint_limits_);
    limit_effort_interface_.registerHandle(limit_handle);
  }
  else
  {
    ROS_ERROR("Verify your command mode setting");
    // ros::shutdown();
    return false;
  }

  return true;
}

void VescHwInterface::read()
{
  // requests joint states
  // function `packetCallback` will be called after receiveing retrun packets
  vesc_interface_.requestState();

  return;
}

void VescHwInterface::read(const ros::Time& time, const ros::Duration& period)
{
  read();
  return;
}

void VescHwInterface::write()
{
  // sends commands
  if (command_mode_ == "position")
  {
    limit_position_interface_.enforceLimits(getPeriod());

    // executes PID control
    servo_controller_.control(command_, position_);
  }
  else if (command_mode_ == "velocity")
  {
    limit_velocity_interface_.enforceLimits(getPeriod());

    // converts the velocity unit: rad/s or m/s -> rpm
    double ref_velocity_rpm = gear_ratio_ * command_ * 60 / (2 * M_PI);

    // sends a reference velocity command
    vesc_interface_.setSpeed(ref_velocity_rpm);
  }
  else if (command_mode_ == "effort")
  {
    limit_effort_interface_.enforceLimits(getPeriod());

    // converts the command unit: Nm or N -> A
    double ref_current = command_ / gear_ratio_ / torque_const_;

    // sends a reference current command
    vesc_interface_.setCurrent(ref_current);
  }
  else if (command_mode_ == "effort_duty")
  {
    command_ = std::max(-1.0, command_);
    command_ = std::min(1.0, command_);

    // sends a  duty command
    vesc_interface_.setDutyCycle(command_);
  }
  return;
}

void VescHwInterface::write(const ros::Time& time, const ros::Duration& period)
{
  write();
  return;
}

ros::Time VescHwInterface::getTime() const
{
  return ros::Time::now();
}

ros::Duration VescHwInterface::getPeriod() const
{
  return ros::Duration(0.01);
}

void VescHwInterface::packetCallback(const boost::shared_ptr<VescPacket const>& packet)
{
  if (packet->getName() == "Values")
  {
    boost::shared_ptr<VescPacketValues const> values = boost::dynamic_pointer_cast<VescPacketValues const>(packet);

    double current = values->getMotorCurrent();
    double velocity_rpm = values->getRpm();
    double position_pulse = values->getPosition();

    position_ = position_pulse / gear_ratio_ - servo_controller_.getZeroPosition();  // unit: rad or m
    velocity_ = velocity_rpm * 2 * M_PI / 60.0 / gear_ratio_;                        // unit: rad/s or m/s
    effort_ = current * torque_const_ * gear_ratio_;                                 // unit: Nm or N
  }

  return;
}

void VescHwInterface::errorCallback(const std::string& error)
{
  ROS_ERROR("%s", error.c_str());
  return;
}

}  // namespace vesc_hw_interface

PLUGINLIB_EXPORT_CLASS(vesc_hw_interface::VescHwInterface, hardware_interface::RobotHW)
