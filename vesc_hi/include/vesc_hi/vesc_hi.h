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

#ifndef VESC_HI_H_
#define VESC_HI_H_

#include <string>
#include <cmath>

#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <serial/serial.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_manager/controller_manager.h>

#include "vesc_driver/vesc_packet.h"
#include "vesc_driver/vesc_packet_factory.h"
#include "vesc_driver/vesc_interface.h"
#include "vesc_hi/vesc_servo_controller.h"

namespace vesc_hi {

using vesc_driver::VescPacket;
using vesc_driver::VescPacketValues;
using vesc_driver::VescInterface;

class VescHI : public hardware_interface::RobotHW {
public:
    explicit VescHI(ros::NodeHandle);
    ~VescHI();

    void          read();
    void          write();
    ros::Time     getTime() const;
    ros::Duration getPeriod() const;

private:
    VescInterface       vesc_interface_;
    VescServoController servo_controller_;

    std::string         joint_name_, command_mode_;

    double command_;
    double position_, velocity_, effort_;  // joint states

    double zero_position_val_;          // sensor value on zero position
    double gear_ratio_, torque_const_;  // physical params.

    hardware_interface::JointStateInterface    joint_state_interface_;
    hardware_interface::PositionJointInterface joint_position_interface_;
    hardware_interface::VelocityJointInterface joint_velocity_interface_;
    hardware_interface::EffortJointInterface   joint_effort_interface_;

    void packetCallback(const boost::shared_ptr<VescPacket const>&);
    void errorCallback(const std::string&);
};

}  // namespace vesc_hi

#endif  // VESC_HI_H_
