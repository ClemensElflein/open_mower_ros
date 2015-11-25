#ifndef VESC_DRIVER_VESC_DRIVER_H
#define VESC_DRIVER_VESC_DRIVER_H

#include <ros/ros.h>

namespace vesc_driver
{

class VescDriver
{
public:

  VescDriver(ros::NodeHandle nh,
             ros::NodeHandle private_nh);

private:

};

} // namespace vesc_driver

#endif // VESC_DRIVER_VESC_DRIVER_H
