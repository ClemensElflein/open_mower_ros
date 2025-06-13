// Created by Clemens Elflein on 22.02.22.
// Copyright (c) 2022 Clemens Elflein and OpenMower contributors. All rights reserved.
//
// This file is part of OpenMower.
//
// OpenMower is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, version 3 of the License.
//
// OpenMower is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
// PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with
// OpenMower. If not, see <https://www.gnu.org/licenses/>.
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "mower_map/GetMowingAreaSrv.h"
#include "ros/ros.h"
#include "slic3r_coverage_planner/PlanPath.h"
#include "xbot_msgs/AbsolutePose.h"

ros::Publisher pose_pub;
geometry_msgs::PoseWithCovarianceStamped out;
std::string frame;

void pose_received(const xbot_msgs::AbsolutePose::ConstPtr &msg) {
  out.header = msg->header;
  out.pose = msg->pose;
  out.pose.pose.position.z = 0;
  out.header.frame_id = frame;
  pose_pub.publish(out);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "xbot_pose_converter");

  ros::NodeHandle n;
  ros::NodeHandle paramNh("~");

  std::string topic;
  if (!paramNh.getParam("topic", topic)) {
    ROS_ERROR_STREAM("You need to prive a topic to convert");
    return 1;
  }
  paramNh.param("frame", frame, std::string("frame"));

  std::string target_topic = topic + "/converted";

  ROS_INFO_STREAM("Converting " << topic << " to " << target_topic);

  pose_pub = paramNh.advertise<geometry_msgs::PoseWithCovarianceStamped>(target_topic, 10, false);

  ros::Subscriber s = n.subscribe(topic, 0, pose_received);

  ros::spin();

  return 0;
}
