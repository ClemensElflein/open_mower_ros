//
// Created by Clemens Elflein on 22.02.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//
#include "ros/ros.h"
#include "slic3r_coverage_planner/PlanPath.h"
#include "mower_map/GetMowingAreaSrv.h"
#include "xbot_msgs/AbsolutePose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

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
    if(!paramNh.getParam("topic", topic)) {
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

