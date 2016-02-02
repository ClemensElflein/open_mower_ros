// -*- mode:c++; fill-column: 100; -*-

#ifndef VESC_ACKERMANN_VESC_TO_ODOM_H_
#define VESC_ACKERMANN_VESC_TO_ODOM_H_

#include <ros/ros.h>
#include <vesc_msgs/VescStateStamped.h>
#include <std_msgs/Float64.h>
#include <boost/shared_ptr.hpp>
#include <tf/transform_broadcaster.h>

namespace vesc_ackermann
{

class VescToOdom
{
public:

  VescToOdom(ros::NodeHandle nh, ros::NodeHandle private_nh);

private:
  // ROS parameters
  std::string odom_frame_;
  std::string base_frame_;
  /** State message does not report servo position, so use the command instead */
  bool use_servo_cmd_;
  // conversion gain and offset
  double speed_to_erpm_gain_, speed_to_erpm_offset_;
  double steering_to_servo_gain_, steering_to_servo_offset_;
  double wheelbase_;
  bool publish_tf_;

  // odometry state
  double x_, y_, yaw_;
  std_msgs::Float64::ConstPtr last_servo_cmd_; ///< Last servo position commanded value
  vesc_msgs::VescStateStamped::ConstPtr last_state_; ///< Last received state message

  // ROS services
  ros::Publisher odom_pub_;
  ros::Subscriber vesc_state_sub_;
  ros::Subscriber servo_sub_;
  boost::shared_ptr<tf::TransformBroadcaster> tf_pub_;

  // ROS callbacks
  void vescStateCallback(const vesc_msgs::VescStateStamped::ConstPtr& state);
  void servoCmdCallback(const std_msgs::Float64::ConstPtr& servo);
};

} // namespace vesc_ackermann

#endif // VESC_ACKERMANN_VESC_TO_ODOM_H_
