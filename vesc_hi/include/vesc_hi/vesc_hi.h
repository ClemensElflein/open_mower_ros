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

#include <ros/ros.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_manager/controller_manager.h>

#include "vesc_driver/vesc_driver.h"

class VescHI : public hardware_interface::RobotHW {
public:
    explicit VescHI(ros::NodeHandle, vesc_driver::VescDriver*);
    ~VescHI();

    void          read();
    void          write();
    ros::Time     getTime() const;
    ros::Duration getPeriod() const;

private:
    std::string              joint_name_;
    vesc_driver::VescDriver* driver_ptr_;
};

#endif
