#include "xesc_2040_driver/xesc_2040_driver.h"


void xesc_2040_driver::Xesc2040Driver::error_func(const std::string &s) {
    ROS_ERROR_STREAM(s);
}


xesc_2040_driver::Xesc2040Driver::Xesc2040Driver(ros::NodeHandle &nh, ros::NodeHandle &private_nh) {
    ROS_INFO_STREAM("Starting xesc 2040 driver");

    xesc_interface = new xesc_2040_driver::Xesc2040Interface(boost::bind(&Xesc2040Driver::error_func, this, boost::placeholders::_1));

    uint8_t hall_table[8];
    float motor_current_limit;
    float acceleration;
    bool has_motor_temp;
    float min_motor_temp;
    float max_motor_temp;
    float min_pcb_temp;
    float max_pcb_temp;
    std::string serial_port;

    if(!private_nh.getParam("serial_port", serial_port)) {
        ROS_ERROR_STREAM("You need to provide parameter serial_port.");
        throw ros::InvalidParameterException("You need to provide parameter serial_port.");
    }

    for(int i = 0; i < 8; i++) {
        int tmp;
        if (!private_nh.getParam(std::string("hall_table_")+std::to_string(i), tmp)) {
            ROS_ERROR_STREAM("You need to provide parameter "<< std::string("hall_table_")+std::to_string(i));
            throw ros::InvalidParameterException("You need to provide parameter "+ std::string("hall_table_")+std::to_string(i));
        }
        hall_table[i] = tmp;
    }

    if(!private_nh.getParam("motor_current_limit",motor_current_limit)) {
        ROS_ERROR_STREAM("You need to provide parameter motor_current_limit");
        throw ros::InvalidParameterException("You need to provide parameter motor_current_limit");
    }
    if(!private_nh.getParam("acceleration",acceleration)) {
        ROS_ERROR_STREAM("You need to provide parameter acceleration");
        throw ros::InvalidParameterException("You need to provide parameter acceleration");
    }
    if(!private_nh.getParam("has_motor_temp",has_motor_temp)) {
        ROS_ERROR_STREAM("You need to provide parameter has_motor_temp");
        throw ros::InvalidParameterException("You need to provide parameter has_motor_temp");
    }
    if(!private_nh.getParam("min_motor_temp",min_motor_temp)) {
        ROS_ERROR_STREAM("You need to provide parameter min_motor_temp");
        throw ros::InvalidParameterException("You need to provide parameter min_motor_temp");
    }
    if(!private_nh.getParam("max_motor_temp",max_motor_temp)) {
        ROS_ERROR_STREAM("You need to provide parameter max_motor_temp");
        throw ros::InvalidParameterException("You need to provide parameter max_motor_temp");
    }
    if(!private_nh.getParam("min_pcb_temp",min_pcb_temp)) {
        ROS_ERROR_STREAM("You need to provide parameter min_pcb_temp");
        throw ros::InvalidParameterException("You need to provide parameter min_pcb_temp");
    }
    if(!private_nh.getParam("max_pcb_temp",max_pcb_temp)) {
        ROS_ERROR_STREAM("You need to provide parameter max_pcb_temp");
        throw ros::InvalidParameterException("You need to provide parameter max_pcb_temp");
    }

    xesc_interface->update_settings(hall_table,
                                    motor_current_limit,
                                    acceleration,
                                    has_motor_temp,
                                    min_motor_temp,
                                    max_motor_temp,
                                    min_pcb_temp,
                                    max_pcb_temp);

    xesc_interface->start(private_nh.param("serial_port", serial_port));
}

void xesc_2040_driver::Xesc2040Driver::getStatus(xesc_msgs::XescStateStamped &state_msg) {
    if(!xesc_interface)
        return;
    xesc_interface->get_status(&status);

    state_msg.header.stamp = ros::Time::now();
    state_msg.state.connection_state = status.connection_state;
    state_msg.state.fw_major = status.fw_version_major;
    state_msg.state.fw_minor = status.fw_version_minor;
    state_msg.state.voltage_input = status.voltage_input;
    state_msg.state.temperature_pcb = status.temperature_pcb;
    state_msg.state.temperature_motor = status.temperature_motor;
    state_msg.state.current_input = status.current_input;
    state_msg.state.duty_cycle = status.duty_cycle;
    state_msg.state.tacho = status.tacho;
    state_msg.state.tacho_absolute = status.tacho_absolute;
    state_msg.state.direction = status.direction;
    state_msg.state.fault_code = status.fault_code;
}

void xesc_2040_driver::Xesc2040Driver::getStatusBlocking(xesc_msgs::XescStateStamped &state_msg) {
    if(!xesc_interface)
        return;
    xesc_interface->wait_for_status(&status);

    state_msg.header.stamp = ros::Time::now();
    state_msg.state.connection_state = status.connection_state;
    state_msg.state.fw_major = status.fw_version_major;
    state_msg.state.fw_minor = status.fw_version_minor;
    state_msg.state.voltage_input = status.voltage_input;
    state_msg.state.temperature_pcb = status.temperature_pcb;
    state_msg.state.temperature_motor = status.temperature_motor;
    state_msg.state.current_input = status.current_input;
    state_msg.state.duty_cycle = status.duty_cycle;
    state_msg.state.tacho = status.tacho;
    state_msg.state.tacho_absolute = status.tacho_absolute;
    state_msg.state.direction = status.direction;
    state_msg.state.fault_code = status.fault_code;
}

void xesc_2040_driver::Xesc2040Driver::stop() {
    ROS_INFO_STREAM("stopping XESC2040 driver");
    xesc_interface->stop();

    delete xesc_interface;
}

void xesc_2040_driver::Xesc2040Driver::setDutyCycle(float duty_cycle) {
    if(xesc_interface) {
        xesc_interface->setDutyCycle(duty_cycle);
    }
}
