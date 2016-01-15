// -*- mode:c++; fill-column: 100; -*-

#ifndef VESC_ACKERMANN_ACKERMANN_TO_VESC_H_
#define VESC_ACKERMANN_ACKERMANN_TO_VESC_H_

#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

namespace vesc_ackermann
{

class AckermannToVesc
{
public:

  AckermannToVesc(ros::NodeHandle nh, ros::NodeHandle private_nh);

private:
  // ROS parameters
  // conversion gain and offset
  double speed_to_erpm_gain_, speed_to_erpm_offset_;
  double steering_to_servo_gain_, steering_to_servo_offset_;

  /** @todo consider also providing an interpolated look-up table conversion */

  // ROS services
  ros::Publisher erpm_pub_;
  ros::Publisher servo_pub_;
  ros::Subscriber ackermann_sub_;

  // ROS callbacks
  void ackermannCmdCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& cmd);
};

} // namespace vesc_ackermann

#endif // VESC_ACKERMANN_ACKERMANN_TO_VESC_H_
