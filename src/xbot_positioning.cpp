//
// Created by Clemens Elflein on 27.10.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//

#include "SystemModel.hpp"

#include "ros/ros.h"

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include "xbot_msgs/AbsolutePose.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "xbot_positioning_core.h"
#include "xbot_msgs/WheelTick.h"

ros::Publisher odometry_pub_predict_only;
ros::Publisher odometry_pub;

xbot::positioning::xbot_positioning_core core;
xbot::positioning::xbot_positioning_core predict_only_core;

bool has_ticks;
xbot_msgs::WheelTick last_ticks;

sensor_msgs::Imu last_imu;
bool has_gyro;
ros::Time gyro_calibration_start;
double gyro_offset;
int gyro_offset_samples;
double vx = 0.0;

nav_msgs::Odometry odometry, odometry_predict_only;

void onImu(const sensor_msgs::Imu::ConstPtr &msg) {
    if(!has_gyro) {
        if(gyro_offset_samples == 0) {
            ROS_INFO_STREAM("Started gyro calibration");
            gyro_calibration_start = msg->header.stamp;
            gyro_offset = 0;
        }
        gyro_offset += msg->angular_velocity.z;
        gyro_offset_samples++;
        if((msg->header.stamp - gyro_calibration_start).toSec() < 5) {
            last_imu = *msg;
            return;
        }
        has_gyro = true;
        if(gyro_offset_samples > 0) {
            gyro_offset /= gyro_offset_samples;
        } else {
            gyro_offset = 0;
        }
        gyro_offset_samples = 0;
        ROS_INFO_STREAM("Calibrated gyro offset: " << gyro_offset);
    }

    auto x = core.predict(vx, msg->angular_velocity.z - gyro_offset, (last_imu.header.stamp - msg->header.stamp).toSec());
    auto x_predict_only = predict_only_core.predict(vx, msg->angular_velocity.z - gyro_offset, (last_imu.header.stamp - msg->header.stamp).toSec());

    odometry.header.stamp = ros::Time::now();
    odometry.header.seq++;
    odometry.header.frame_id = "map";
    odometry.child_frame_id = "base_link";
    odometry.pose.pose.position.x = x.x_pos();
    odometry.pose.pose.position.y = x.y_pos();
    tf2::Quaternion q(0.0, 0.0, x.theta());
    odometry.pose.pose.orientation = tf2::toMsg(q);

    odometry_predict_only.header = odometry.header;
    odometry_predict_only.child_frame_id = odometry.child_frame_id;
    odometry_predict_only.pose.pose.position.x = x_predict_only.x_pos();
    odometry_predict_only.pose.pose.position.y = x_predict_only.y_pos();
    tf2::Quaternion q_predict_only(0.0, 0.0, x_predict_only.theta());
    odometry_predict_only.pose.pose.orientation = tf2::toMsg(q_predict_only);

    odometry_pub.publish(odometry);
    odometry_pub_predict_only.publish(odometry_predict_only);

    last_imu = *msg;
}

void onWheelTicks(const xbot_msgs::WheelTick::ConstPtr &msg) {
    if(!has_ticks) {
        last_ticks = *msg;
        has_ticks = true;
        return;
    }
    double dt = (msg->stamp - last_ticks.stamp).toSec();

    double d_wheel_l = (double) (msg->wheel_ticks_rl - last_ticks.wheel_ticks_rl) / (993.0 / (0.19*M_PI));
    double d_wheel_r = (double) (msg->wheel_ticks_rr - last_ticks.wheel_ticks_rr) / (993.0 / (0.19*M_PI));

    if(msg->wheel_direction_rl) {
        d_wheel_l *= -1.0;
    }
    if(msg->wheel_direction_rr) {
        d_wheel_r *= -1.0;
    }


    double d_ticks = -(d_wheel_l + d_wheel_r) / 2.0;
    vx = d_ticks / dt;

    last_ticks = *msg;
}

void onPose(const xbot_msgs::AbsolutePose::ConstPtr &msg) {
    core.updatePosition(msg->pose.pose.position.x, msg->pose.pose.position.y);
    double c = 100000.0*pow(last_imu.angular_velocity.z - gyro_offset,2) + 10000.0;
    core.updateOrientation(msg->motion_heading, c);
    ROS_INFO_STREAM("c:" << c);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "xbot_positioning");

    vx = 0.0;
    has_gyro = false;
    has_ticks = false;
    gyro_offset = 0;
    gyro_offset_samples = 0;

    ros::NodeHandle n;
    ros::NodeHandle paramNh("~");

    odometry_pub = paramNh.advertise<nav_msgs::Odometry>("odom", 50);
    odometry_pub_predict_only = paramNh.advertise<nav_msgs::Odometry>("odom_predict_only", 50);

    ros::Subscriber imu_sub = paramNh.subscribe("imu", 10, onImu);
    ros::Subscriber pose_sub = paramNh.subscribe("xb_pose", 10, onPose);
    ros::Subscriber wheel_tick_sub = paramNh.subscribe("wheel_ticks", 10, onWheelTicks);

    ros::spin();
    return 0;
}