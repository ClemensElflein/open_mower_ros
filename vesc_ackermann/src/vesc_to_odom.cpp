// -*- mode:c++; fill-column: 100; -*-

#include "vesc_ackermann/vesc_to_odom.h"

#include <cmath>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

namespace vesc_ackermann
{

template <typename T>
inline bool getRequiredParam(const ros::NodeHandle& nh, std::string name, T& value);

VescToOdom::VescToOdom(ros::NodeHandle nh, ros::NodeHandle private_nh) :
  odom_frame_("odom"), base_frame_("base_link"),
  use_servo_cmd_(true), publish_tf_(false), x_(0.0), y_(0.0), yaw_(0.0)
{
  // get ROS parameters
  private_nh.param("odom_frame", odom_frame_, odom_frame_);
  private_nh.param("base_frame", base_frame_, base_frame_);
  private_nh.param("use_servo_cmd_to_calc_angular_velocity", use_servo_cmd_, use_servo_cmd_);
  if (!getRequiredParam(nh, "speed_to_erpm_gain", speed_to_erpm_gain_))
    return;
  if (!getRequiredParam(nh, "speed_to_erpm_offset", speed_to_erpm_offset_))
    return;
  if (use_servo_cmd_) {
    if (!getRequiredParam(nh, "steering_angle_to_servo_gain", steering_to_servo_gain_))
      return;
    if (!getRequiredParam(nh, "steering_angle_to_servo_offset", steering_to_servo_offset_))
      return;
    if (!getRequiredParam(nh, "wheelbase", wheelbase_))
      return;
  }
  private_nh.param("publish_tf", publish_tf_, publish_tf_);

  // create odom publisher
  odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 10);

  // create tf broadcaster
  if (publish_tf_) {
    tf_pub_.reset(new tf::TransformBroadcaster);
  }

  // subscribe to vesc state and. optionally, servo command
  vesc_state_sub_ = nh.subscribe("sensors/core", 10, &VescToOdom::vescStateCallback, this);
  if (use_servo_cmd_) {
    servo_sub_ = nh.subscribe("sensors/servo_position_command", 10,
                              &VescToOdom::servoCmdCallback, this);
  }
}

void VescToOdom::vescStateCallback(const vesc_msgs::VescStateStamped::ConstPtr& state)
{
  // check that we have a last servo command if we are depending on it for angular velocity
  if (use_servo_cmd_ && !last_servo_cmd_)
    return;

  // convert to engineering units
  double current_speed = ( state->state.speed - speed_to_erpm_offset_ ) / speed_to_erpm_gain_;
  double current_steering_angle(0.0), current_angular_velocity(0.0);
  if (use_servo_cmd_) {
    current_steering_angle =
      ( last_servo_cmd_->data - steering_to_servo_offset_ ) / steering_to_servo_gain_;
    current_angular_velocity = current_speed * tan(current_steering_angle) / wheelbase_;
  }

  // use current state as last state if this is our first time here
  if (!last_state_)
    last_state_ = state;

  // calc elapsed time
  ros::Duration dt = state->header.stamp - last_state_->header.stamp;

  /** @todo could probably do better propigating odometry, e.g. trapezoidal integration */

  // propigate odometry
  double x_dot = current_speed * cos(yaw_);
  double y_dot = current_speed * sin(yaw_);
  x_ += x_dot * dt.toSec();
  y_ += y_dot * dt.toSec();
  if (use_servo_cmd_)
    yaw_ += current_angular_velocity * dt.toSec();

  // save state for next time
  last_state_ = state;

  // publish odometry message
  nav_msgs::Odometry::Ptr odom(new nav_msgs::Odometry);
  odom->header.frame_id = odom_frame_;
  odom->header.stamp = state->header.stamp;
  odom->child_frame_id = base_frame_;

  // Position
  odom->pose.pose.position.x = x_;
  odom->pose.pose.position.y = y_;
  odom->pose.pose.orientation.x = 0.0;
  odom->pose.pose.orientation.y = 0.0;
  odom->pose.pose.orientation.z = sin(yaw_/2.0);
  odom->pose.pose.orientation.w = cos(yaw_/2.0);

  // Position uncertainty
  /** @todo Think about position uncertainty, perhaps get from parameters? */
  odom->pose.covariance[0]  = 0.2; ///< x
  odom->pose.covariance[7]  = 0.2; ///< y
  odom->pose.covariance[35] = 0.4; ///< yaw

  // Velocity ("in the coordinate frame given by the child_frame_id")
  odom->twist.twist.linear.x = current_speed;
  odom->twist.twist.linear.y = 0.0;
  odom->twist.twist.angular.z = current_angular_velocity;

  // Velocity uncertainty
  /** @todo Think about velocity uncertainty */

  if (publish_tf_) {
    geometry_msgs::TransformStamped tf;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    tf.header.stamp = ros::Time::now();
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = odom->pose.pose.orientation;
    if (ros::ok()) {
      tf_pub_->sendTransform(tf);
    }
  }

  if (ros::ok()) {
    odom_pub_.publish(odom);
  }
}

void VescToOdom::servoCmdCallback(const std_msgs::Float64::ConstPtr& servo)
{
  last_servo_cmd_ = servo;
}

template <typename T>
inline bool getRequiredParam(const ros::NodeHandle& nh, std::string name, T& value)
{
  if (nh.getParam(name, value))
    return true;

  ROS_FATAL("VescToOdom: Parameter %s is required.", name.c_str());
  return false;
}

} // namespace vesc_ackermann
