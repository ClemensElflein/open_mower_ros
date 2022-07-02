#include <ros/ros.h>

#include "xesc_2040_driver/xesc_2040_interface.h"
#include "geometry_msgs/Twist.h"
#include <vesc_msgs/VescStateStamped.h>

void error_func(const std::string &s) {
    ROS_ERROR_STREAM(s);
}

xesc_2040_driver::Xesc2040Interface* xesc_interface = nullptr;

void velReceived(const geometry_msgs::Twist::ConstPtr &msg) {
    if(!xesc_interface)
        return;

    xesc_interface->setDutyCycle(msg->linear.x*2);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "xesc_2040_driver_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 0, velReceived, ros::TransportHints().tcpNoDelay(true));
    xesc_interface = new xesc_2040_driver::Xesc2040Interface(error_func);



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
        return 1;
    }

    for(int i = 0; i < 8; i++) {
        int tmp;
        if (!private_nh.getParam(std::string("hall_table_")+std::to_string(i), tmp)) {
            ROS_ERROR_STREAM("You need to provide parameter "<< std::string("hall_table_")+std::to_string(i));
            return 1;
        }
        hall_table[i] = tmp;
    }

    if(!private_nh.getParam("motor_current_limit",motor_current_limit)) {
        ROS_ERROR_STREAM("You need to provide parameter motor_current_limit");
        return 1;
    }
    if(!private_nh.getParam("acceleration",acceleration)) {
        ROS_ERROR_STREAM("You need to provide parameter acceleration");
        return 1;
    }
    if(!private_nh.getParam("has_motor_temp",has_motor_temp)) {
        ROS_ERROR_STREAM("You need to provide parameter has_motor_temp");
        return 1;
    }
    if(!private_nh.getParam("min_motor_temp",min_motor_temp)) {
        ROS_ERROR_STREAM("You need to provide parameter min_motor_temp");
        return 1;
    }
    if(!private_nh.getParam("max_motor_temp",max_motor_temp)) {
        ROS_ERROR_STREAM("You need to provide parameter max_motor_temp");
        return 1;
    }
    if(!private_nh.getParam("min_pcb_temp",min_pcb_temp)) {
        ROS_ERROR_STREAM("You need to provide parameter min_pcb_temp");
        return 1;
    }
    if(!private_nh.getParam("max_pcb_temp",max_pcb_temp)) {
        ROS_ERROR_STREAM("You need to provide parameter max_pcb_temp");
        return 1;
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



    // create vesc state (telemetry) publisher
    ros::Publisher state_pub = nh.advertise<vesc_msgs::VescStateStamped>("sensors/core", 10);

    ROS_INFO_STREAM("started XESC2040 driver");



    ros::AsyncSpinner spinner(1);
    spinner.start();
    xesc_2040_driver::Xesc2040StatusStruct status;
    vesc_msgs::VescStateStamped state_msg;
    while (ros::ok()) {
        xesc_interface->wait_for_status(&status);
        
        state_msg.header.stamp = ros::Time::now();
        state_msg.state.connection_state = status.connection_state;
        state_msg.state.fw_major = status.fw_version_major;
        state_msg.state.fw_minor = status.fw_version_minor;
        state_msg.state.voltage_input = status.voltage_input;
        state_msg.state.temperature_pcb = status.temperature_pcb;
        state_msg.state.current_motor = status.current_input;
        state_msg.state.current_input = status.current_input;
        state_msg.state.speed = 0;
        state_msg.state.duty_cycle = status.duty_cycle;
        state_msg.state.charge_drawn = 0;
        state_msg.state.charge_regen = 0;
        state_msg.state.energy_drawn = 0;
        state_msg.state.energy_regen = 0;
        state_msg.state.displacement = 0;
        state_msg.state.distance_traveled = 0;
        state_msg.state.fault_code = status.fault_code;

        state_pub.publish(state_msg);
    }
    ROS_INFO_STREAM("stopping XESC2040 driver");
    xesc_interface->stop();
    spinner.stop();

    delete xesc_interface;

    return 0;
}
