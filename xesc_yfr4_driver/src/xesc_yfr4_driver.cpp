#include "xesc_yfr4_driver/xesc_yfr4_driver.h"

void xesc_yfr4_driver::XescYFR4Driver::error_func(const std::string &s) {
    ROS_ERROR_STREAM(s);
}

xesc_yfr4_driver::XescYFR4Driver::XescYFR4Driver(ros::NodeHandle &nh, ros::NodeHandle &private_nh) {
    ROS_INFO_STREAM("Starting xesc YardForce R4 adapter driver");

    xesc_interface = new xesc_yfr4_driver::XescYFR4Interface(boost::bind(&XescYFR4Driver::error_func, this, boost::placeholders::_1));

    float motor_current_limit;
    float min_pcb_temp;
    float max_pcb_temp;
    std::string serial_port;

    if (!private_nh.getParam("serial_port", serial_port)) {
        ROS_ERROR_STREAM("You need to provide parameter serial_port.");
        throw ros::InvalidParameterException("You need to provide parameter serial_port.");
    }
    if (!private_nh.getParam("motor_current_limit", motor_current_limit)) {
        ROS_ERROR_STREAM("You need to provide parameter motor_current_limit");
        throw ros::InvalidParameterException("You need to provide parameter motor_current_limit");
    }
    if (!private_nh.getParam("min_pcb_temp", min_pcb_temp)) {
        ROS_ERROR_STREAM("You need to provide parameter min_pcb_temp");
        throw ros::InvalidParameterException("You need to provide parameter min_pcb_temp");
    }
    if (!private_nh.getParam("max_pcb_temp", max_pcb_temp)) {
        ROS_ERROR_STREAM("You need to provide parameter max_pcb_temp");
        throw ros::InvalidParameterException("You need to provide parameter max_pcb_temp");
    }

    xesc_interface->update_settings(motor_current_limit, min_pcb_temp, max_pcb_temp);
    xesc_interface->start(private_nh.param("serial_port", serial_port));
}

void xesc_yfr4_driver::XescYFR4Driver::getStatus(xesc_msgs::XescStateStamped &state_msg) {
    if (!xesc_interface)
        return;

    xesc_interface->get_status(&status);
    state_msg.header.stamp = ros::Time::now();
    state_msg.state.connection_state = status.connection_state;
    state_msg.state.fw_major = status.fw_version_major;
    state_msg.state.fw_minor = status.fw_version_minor;
    state_msg.state.temperature_pcb = status.temperature_pcb;
    state_msg.state.current_input = status.current_input;
    state_msg.state.duty_cycle = status.duty_cycle;
    state_msg.state.direction = status.direction;
    state_msg.state.tacho = status.tacho;
    state_msg.state.tacho_absolute = status.tacho_absolute;
    state_msg.state.rpm = status.rpm;
    state_msg.state.fault_code = status.fault_code;
}

void xesc_yfr4_driver::XescYFR4Driver::getStatusBlocking(xesc_msgs::XescStateStamped &state_msg) {
    if (!xesc_interface)
        return;
    xesc_interface->wait_for_status(&status);
    state_msg.header.stamp = ros::Time::now();
    state_msg.state.connection_state = status.connection_state;
    state_msg.state.fw_major = status.fw_version_major;
    state_msg.state.fw_minor = status.fw_version_minor;
    state_msg.state.temperature_pcb = status.temperature_pcb;
    state_msg.state.current_input = status.current_input;
    state_msg.state.duty_cycle = status.duty_cycle;
    state_msg.state.direction = status.direction;
    state_msg.state.tacho = status.tacho;
    state_msg.state.tacho_absolute = status.tacho_absolute;
    state_msg.state.rpm = status.rpm;
    state_msg.state.fault_code = status.fault_code;
}

void xesc_yfr4_driver::XescYFR4Driver::stop() {
    ROS_INFO_STREAM("Stopping xesc YardForce R4 adapter driver");
    xesc_interface->stop();
    delete xesc_interface;
}

void xesc_yfr4_driver::XescYFR4Driver::setDutyCycle(float duty_cycle) {
    if (xesc_interface) {
        xesc_interface->setDutyCycle(duty_cycle);
    }
}
