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
#include "xbot_positioning/KalmanState.h"

ros::Publisher odometry_pub_predict_only;
ros::Publisher odometry_pub;
ros::Publisher dbg_expected_motion_vector;
ros::Publisher kalman_state;

xbot::positioning::xbot_positioning_core core;
xbot::positioning::xbot_positioning_core predict_only_core;

bool skip_gyro_calibration;
bool has_ticks;
xbot_msgs::WheelTick last_ticks;

sensor_msgs::Imu last_imu;
bool has_gyro;
ros::Time gyro_calibration_start;
double gyro_offset;
int gyro_offset_samples;
double vx = 0.0;
double min_speed = 0.0;

nav_msgs::Odometry odometry, odometry_predict_only;

xbot_positioning::KalmanState state_msg;

void onImu(const sensor_msgs::Imu::ConstPtr &msg) {
    if(!has_gyro) {
        if(!skip_gyro_calibration) {
            if (gyro_offset_samples == 0) {
                ROS_INFO_STREAM("Started gyro calibration");
                gyro_calibration_start = msg->header.stamp;
                gyro_offset = 0;
            }
            gyro_offset += -msg->angular_velocity.z;
            gyro_offset_samples++;
            if ((msg->header.stamp - gyro_calibration_start).toSec() < 5) {
                last_imu = *msg;
                return;
            }
            has_gyro = true;
            if (gyro_offset_samples > 0) {
                gyro_offset /= gyro_offset_samples;
            } else {
                gyro_offset = 0;
            }
            gyro_offset_samples = 0;
            ROS_INFO_STREAM("Calibrated gyro offset: " << gyro_offset);
        } else {
            ROS_WARN("Skipped gyro calibration");
            has_gyro = true;
            return;
        }
    }

    core.predict(vx, -msg->angular_velocity.z - gyro_offset, (msg->header.stamp - last_imu.header.stamp).toSec());
    auto x = core.updateSpeed(vx, -msg->angular_velocity.z - gyro_offset,0.01);
    predict_only_core.predict(vx, -msg->angular_velocity.z - gyro_offset, (msg->header.stamp - last_imu.header.stamp).toSec());
    auto x_predict_only = predict_only_core.updateSpeed(vx, -msg->angular_velocity.z - gyro_offset,0.01);

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

    ROS_INFO_STREAM_THROTTLE(1,"c:\n" << core.getCovariance() << "\n\n" << "state\n:" << core.getState());

    auto state = core.getState();
    state_msg.x = state.x();
    state_msg.y = state.y();
    state_msg.theta = state.theta();
    state_msg.vx = state.vx();
    state_msg.vr = state.vr();

    kalman_state.publish(state_msg);

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

    double d_wheel_l = (double) (msg->wheel_ticks_rl - last_ticks.wheel_ticks_rl) * (1/1600.0);
    double d_wheel_r = (double) (msg->wheel_ticks_rr - last_ticks.wheel_ticks_rr) * (1/1600.0);

    if(msg->wheel_direction_rl) {
        d_wheel_l *= -1.0;
    }
    if(msg->wheel_direction_rr) {
        d_wheel_r *= -1.0;
    }


    double d_ticks = (d_wheel_l + d_wheel_r) / 2.0;
    vx = d_ticks / dt;

    last_ticks = *msg;
}

void onPose(const xbot_msgs::AbsolutePose::ConstPtr &msg) {

    core.updatePosition(msg->pose.pose.position.x, msg->pose.pose.position.y);
//    double c = 100000.0*pow(last_imu.angular_velocity.z - gyro_offset,2) + 10000.0;
//    core.updateOrientation(msg->motion_heading, 0.001);
auto m = core.om2.h(core.ekf.getState());
geometry_msgs::Vector3 dbg;
dbg.x = m.vx();
dbg.y = m.vy();
    dbg_expected_motion_vector.publish(dbg);

    if(std::sqrt(std::pow(msg->motion_vector.x, 2)+std::pow(msg->motion_vector.y, 2)) > min_speed) {
        core.updateOrientation2(msg->motion_vector.x, msg->motion_vector.y, 10000.0);
    }
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

    paramNh.param("skip_gyro_calibration", skip_gyro_calibration, false);
    paramNh.param("gyro_offset", gyro_offset, 0.0);
    paramNh.param("min_speed", min_speed, 0.01);

    if(gyro_offset != 0.0) {
        ROS_WARN_STREAM("Using gyro offset of: " << gyro_offset);
    }

    odometry_pub = paramNh.advertise<nav_msgs::Odometry>("odom", 50);
    odometry_pub_predict_only = paramNh.advertise<nav_msgs::Odometry>("odom_predict_only", 50);
    dbg_expected_motion_vector = paramNh.advertise<geometry_msgs::Vector3>("debug_expected_motion_vector", 50);
    kalman_state = paramNh.advertise<xbot_positioning::KalmanState>("kalman_state", 50);

    ros::Subscriber imu_sub = paramNh.subscribe("imu", 10, onImu);
    ros::Subscriber pose_sub = paramNh.subscribe("xb_pose", 10, onPose);
    ros::Subscriber wheel_tick_sub = paramNh.subscribe("wheel_ticks", 10, onWheelTicks);

    ros::spin();
    return 0;
}