#include <ros/ros.h>

#include "std_msgs/Float32.h"
#include <xesc_msgs/XescStateStamped.h>
#include "xesc_driver/xesc_driver.h"

xesc_driver::XescDriver* xesc_driver_ptr = nullptr;

void velReceived(const std_msgs::Float32::ConstPtr &msg) {
    if(!xesc_driver_ptr)
        return;

    xesc_driver_ptr->setDutyCycle(msg->data);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "xesc_2040_driver_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Subscriber duty_cycle_sub = private_nh.subscribe("duty_cycle", 0, velReceived, ros::TransportHints().tcpNoDelay(true));

    // create xesc state (telemetry) publisher
    ros::Publisher state_pub = nh.advertise<xesc_msgs::XescStateStamped>("sensors/core", 10);

    xesc_driver_ptr = new xesc_driver::XescDriver(nh,private_nh);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    xesc_msgs::XescStateStamped state_msg;
    while (ros::ok()) {
        xesc_driver_ptr->getStatusBlocking(state_msg);
        state_pub.publish(state_msg);
    }
    ROS_INFO_STREAM("stopping XESC driver node");
    xesc_driver_ptr->stop();
    spinner.stop();

    delete xesc_driver_ptr;

    return 0;
}
