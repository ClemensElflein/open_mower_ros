#include <ros/ros.h>

#include "vesc_driver.h"

namespace vesc_driver
{

VescDriver::VescDriver(ros::NodeHandle nh,
                       ros::NodeHandle private_nh)
{
  ROS_DEBUG("Constructing VescDriver");
}

} // namespace vesc_driver
