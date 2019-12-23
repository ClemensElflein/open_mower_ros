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

#include "vesc_hi/vesc_hi.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "ocservo_hi");

    ros::NodeHandle         nh, nh_private("~");
    vesc_driver::VescDriver vesc_driver(nh, nh_private);
    VescHI                  vesc_hi(nh_private, &vesc_driver);

    controller_manager::ControllerManager controller_manager(&vesc_hi, nh);

    ros::Rate         loop_rate(1.0 / vesc_hi.getPeriod().toSec());
    ros::AsyncSpinner spinner(1);

    spinner.start();

    while(ros::ok()) {
        // sends commands
        vesc_hi.read();

        // updates the hardware interface control
        controller_manager.update(vesc_hi.getTime(), vesc_hi.getPeriod());

        // gets current states
        vesc_hi.write();

        // sleeps
        loop_rate.sleep();
    }

    spinner.stop();

    return 0;
}

VescHI::VescHI(ros::NodeHandle nh, vesc_driver::VescDriver* driver_ptr) {
    // initializes the private driver pointer
    if(driver_ptr == NULL) {
        ros::shutdown();
    } else {
        driver_ptr_ = driver_ptr;
    }

    // initializes joint names
    nh.param<std::string>("joint_name", joint_name_, "/dev/ttyVESC1");

    // initializes commands
    command_ = 0.0;

    // initializes states
    position_ = 0.0;
    velocity_ = 0.0;
    effort_   = 0.0;

    // defines and registers joint handles
    hardware_interface::JointStateHandle state_handle(joint_name_, &position_, &velocity_, &effort_);
    joint_state_interface_.registerHandle(state_handle);

    hardware_interface::JointHandle position_handle(joint_state_interface_.getHandle(joint_name_), &command_);
    joint_position_interface_.registerHandle(position_handle);

    // registers interfaces
    registerInterface(&joint_state_interface_);
    registerInterface(&joint_position_interface_);
}

VescHI::~VescHI() {
}

void VescHI::read() {
    // sends commands

    return;
}

void VescHI::write() {
    // gets joint states

    return;
}

ros::Time VescHI::getTime() const {
    return ros::Time::now();
}

ros::Duration VescHI::getPeriod() const {
    return ros::Duration(0.01);
}
