#include <ros/ros.h>

#include "vesc_ackermann/vesc_to_odom.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vesc_to_odom_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  vesc_ackermann::VescToOdom vesc_to_odom(nh, private_nh);

  ros::spin();

  return 0;
}
