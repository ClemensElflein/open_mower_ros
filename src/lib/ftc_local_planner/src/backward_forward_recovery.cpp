#include <ftc_local_planner/backward_forward_recovery.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <geometry_msgs/PoseStamped.h>

namespace ftc_local_planner
{

BackwardForwardRecovery::BackwardForwardRecovery() 
  : initialized_(false), 
    max_distance_(0.5),
    linear_vel_(0.3), 
    check_frequency_(10.0), 
    max_cost_threshold_(costmap_2d::INSCRIBED_INFLATED_OBSTACLE-10),
    timeout_(ros::Duration(3.0)) {}

void BackwardForwardRecovery::initialize(std::string name, tf2_ros::Buffer* tf,
                                costmap_2d::Costmap2DROS* global_costmap,
                                costmap_2d::Costmap2DROS* local_costmap)
{
  if(!initialized_){
    name_ = name;
    tf_ = tf;
    global_costmap_ = global_costmap;
    local_costmap_ = local_costmap;

    ros::NodeHandle private_nh("~/" + name_);
    cmd_vel_pub_ = private_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    private_nh.param("max_distance", max_distance_, 0.5);
    private_nh.param("linear_vel", linear_vel_, 0.3);
    private_nh.param("check_frequency", check_frequency_, 10.0);
    int temp_threshold;
    private_nh.param("max_cost_threshold", temp_threshold, static_cast<int>(costmap_2d::INSCRIBED_INFLATED_OBSTACLE-10));
    max_cost_threshold_ = static_cast<unsigned char>(temp_threshold);

    double timeout_seconds;
    private_nh.param("timeout", timeout_seconds, 3.0);
    timeout_ = ros::Duration(timeout_seconds);

    initialized_ = true;
  }
  else{
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

void BackwardForwardRecovery::runBehavior()
{
  if (!initialized_)
  {
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  ROS_WARN("Running Backward/Forward recovery behavior");

  if (attemptMove(max_distance_, false)) {
    ROS_INFO("Successfully moved backwards");
    return;
  }

  if (attemptMove(max_distance_, true)) {
    ROS_INFO("Successfully moved forwards");
    return;
  }

  ROS_WARN("Backward/Forward recovery behavior failed to move in either direction");
}

bool BackwardForwardRecovery::attemptMove(double distance, bool forward)
{
  geometry_msgs::PoseStamped start_pose;
  local_costmap_->getRobotPose(start_pose);

  ros::Rate rate(check_frequency_);
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = forward ? linear_vel_ : -linear_vel_;

  double moved_distance = 0.0;
  ros::Time start_time = ros::Time::now();
  while (moved_distance < distance && (ros::Time::now() - start_time) < timeout_)
  {
    geometry_msgs::PoseStamped current_pose;
    local_costmap_->getRobotPose(current_pose);

    moved_distance = std::hypot(
      current_pose.pose.position.x - start_pose.pose.position.x,
      current_pose.pose.position.y - start_pose.pose.position.y
    );

    if (!isPositionValid(current_pose.pose.position.x, current_pose.pose.position.y))
    {
      ROS_WARN("Reached maximum allowed cost after moving %.2f meters", moved_distance);
      cmd_vel.linear.x = 0;
      cmd_vel_pub_.publish(cmd_vel);
      return false;
    }

    cmd_vel_pub_.publish(cmd_vel);
    rate.sleep();
  }

  cmd_vel.linear.x = 0;
  cmd_vel_pub_.publish(cmd_vel);

  if (moved_distance >= distance) {
    ROS_INFO("%s movement completed successfully", forward ? "Forward" : "Backward");
    return true;
  } else {
    ROS_WARN("%s movement timed out after %.2f seconds", forward ? "Forward" : "Backward", timeout_.toSec());
    return false;
  }
}

bool BackwardForwardRecovery::isPositionValid(double x, double y)
{
  unsigned int mx, my;
  if (local_costmap_->getCostmap()->worldToMap(x, y, mx, my))
  {
    unsigned char cost = local_costmap_->getCostmap()->getCost(mx, my);
    return (cost <= max_cost_threshold_);
  }
  return false;
}

}  // namespace ftc_local_planner

PLUGINLIB_EXPORT_CLASS(ftc_local_planner::BackwardForwardRecovery, nav_core::RecoveryBehavior)