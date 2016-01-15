#include <ros/ros.h>

#include "vesc_ackermann/ackermann_to_vesc.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ackermann_to_vesc_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  vesc_ackermann::AckermannToVesc ackermann_to_vesc(nh, private_nh);

  ros::spin();

  return 0;
}
