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
    ros::init(argc, argv, "vesc_hi");

    ros::NodeHandle nh, nh_private("~");
    vesc_hi::VescHI vesc_hi(nh_private);

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

namespace vesc_hi {

VescHI::VescHI(ros::NodeHandle nh)
    : vesc_interface_(std::string(), boost::bind(&VescHI::packetCallback, this, _1),
                      boost::bind(&VescHI::errorCallback, this, _1)) {
    // reads a port name to open
    std::string port;
    if(!nh.getParam("port", port)) {
        ROS_FATAL("VESC communication port parameter required.");
        ros::shutdown();
        return;
    }

    // attempts to open the serial port
    try {
        vesc_interface_.connect(port);
    } catch(serial::SerialException exception) {
        ROS_FATAL("Failed to connect to the VESC, %s.", exception.what());
        ros::shutdown();
        return;
    }

    // initializes joint names
    nh.param<std::string>("joint_name", joint_name_, "joint_vesc");

    // initializes commands and states
    command_  = 0.0;
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

void VescHI::packetCallback(const boost::shared_ptr<VescPacket const>& packet) {
    if(packet->getName() == "Values") {
        boost::shared_ptr<VescPacketValues const> values = boost::dynamic_pointer_cast<VescPacketValues const>(packet);

        // state_msg->header.stamp            = ros::Time::now();
        double current        = values->getMotorCurrent();
        double velocity_rpm   = values->getRpm();
        double position_pulse = values->getPosition();
    }

    return;
}

void VescHI::errorCallback(const std::string& error) {
    ROS_ERROR("%s", error.c_str());
    return;
}

}  // namespace vesc_hi
