//
// Created by Clemens Elflein on 27.10.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>

#include "SystemModel.hpp"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "ros/ros.h"
#include "xbot_msgs/AbsolutePose.h"
#include "xbot_msgs/WheelTick.h"
#include "xbot_positioning/GPSControlSrv.h"
#include "xbot_positioning/KalmanState.h"
#include "xbot_positioning/SetPoseSrv.h"
#include "xbot_positioning_core.h"

ros::Publisher odometry_pub;
ros::Publisher xbot_absolute_pose_pub;

// Debug Publishers
ros::Publisher kalman_state;
ros::Publisher dbg_expected_motion_vector;

// The kalman filters
xbot::positioning::xbot_positioning_core core{};

// True, if we don't want to do gyro calibration on launch
bool skip_gyro_calibration;

// True, if we have wheel ticks (i.e. last_ticks is valid)
bool has_ticks;
xbot_msgs::WheelTick last_ticks;
bool has_gps;
xbot_msgs::AbsolutePose last_gps;

// True, if last_imu is valid and gyro_offset is valid
bool has_gyro;
sensor_msgs::Imu last_imu;
ros::Time gyro_calibration_start;
double gyro_offset;
int gyro_offset_samples;

// Current speed calculated by wheel ticks
double vx = 0.0;

// Min speed for motion vector to be fed into kalman filter
double min_speed = 0.0;

// Max position accuracy to allow for GPS updates
double max_gps_accuracy;

// Process noise, as variance per second (see xbot_positioning_core::setProcessNoise)
double q_xy, q_theta, q_vx, q_vr;

// Measurement noise for the wheel-odometry speed / gyro rate update
double r_vx, r_vr;

// The GPS reports its own accuracy; these turn that into a measurement variance.
// The reported value is scaled (receivers are habitually optimistic) and floored
// (so one over-confident epoch cannot snap the state).
double gps_accuracy_scale, gps_accuracy_floor;

// Measurement noise on each component of the GPS motion vector, (m/s)^2
double orientation_r;

// Largest prediction step we will take. A dropout or a restart must not be
// allowed to propagate the state by an arbitrary amount.
double max_predict_dt;

// True, if we should publish debug topics (expected motion vector and kalman state)
bool publish_debug;

// Antenna offset (offset between point of rotation and antenna)
double antenna_offset_x, antenna_offset_y;

nav_msgs::Odometry odometry;
xbot_positioning::KalmanState state_msg;
xbot_msgs::AbsolutePose xb_absolute_pose_msg;

bool gps_enabled = true;
int gps_outlier_count = 0;
int valid_gps_samples = 0;

ros::Time last_gps_time(0.0);


void onImu(const sensor_msgs::Imu::ConstPtr &msg) {
    if (!has_gyro) {
        if (!skip_gyro_calibration) {
            if (gyro_offset_samples == 0) {
                ROS_INFO_STREAM("Started gyro calibration");
                gyro_calibration_start = msg->header.stamp;
                gyro_offset = 0;
            }
            gyro_offset += msg->angular_velocity.z;
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
            // Record this sample so the next callback has a sane dt. Without it
            // last_imu.header.stamp is still zero and the first prediction step
            // propagates the state by ~1.7e9 seconds.
            last_imu = *msg;
            return;
        }
    }

    const double dt = (msg->header.stamp - last_imu.header.stamp).toSec();
    if (dt <= 0.0 || dt > max_predict_dt) {
        ROS_WARN_STREAM_THROTTLE(1, "Skipping prediction, implausible dt of " << dt << "s");
        last_imu = *msg;
        return;
    }

    core.predict(vx, msg->angular_velocity.z - gyro_offset, dt);
    auto x = core.updateSpeed(vx, msg->angular_velocity.z - gyro_offset);

    odometry.header.stamp = msg->header.stamp;
    odometry.header.seq++;
    odometry.header.frame_id = "map";
    odometry.child_frame_id = "base_link";
    odometry.pose.pose.position.x = x.x_pos();
    odometry.pose.pose.position.y = x.y_pos();
    odometry.pose.pose.position.z = 0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, x.theta());
    odometry.pose.pose.orientation = tf2::toMsg(q);

    odometry.twist.twist.linear.x = x.vx();
    odometry.twist.twist.angular.z = x.vr();

    {
        const auto &P = core.getCovariance();
        // nav_msgs/Odometry covariance is row-major [x y z roll pitch yaw].
        odometry.pose.covariance[0] = P(xbot::positioning::StateT::X, xbot::positioning::StateT::X);
        odometry.pose.covariance[1] = P(xbot::positioning::StateT::X, xbot::positioning::StateT::Y);
        odometry.pose.covariance[6] = P(xbot::positioning::StateT::Y, xbot::positioning::StateT::X);
        odometry.pose.covariance[7] = P(xbot::positioning::StateT::Y, xbot::positioning::StateT::Y);
        odometry.pose.covariance[35] = P(xbot::positioning::StateT::THETA, xbot::positioning::StateT::THETA);
        // Position and heading errors are strongly correlated while driving (the
        // motion model propagates position along theta), so publish the cross
        // terms rather than leaving them at zero and claiming independence.
        // Nothing in this repo reads them today - odom_out has no in-tree
        // subscriber - but a half-filled covariance is indistinguishable from a
        // genuinely uncorrelated one for anything that picks the topic up later.
        odometry.pose.covariance[5] = P(xbot::positioning::StateT::X, xbot::positioning::StateT::THETA);
        odometry.pose.covariance[11] = P(xbot::positioning::StateT::Y, xbot::positioning::StateT::THETA);
        odometry.pose.covariance[30] = P(xbot::positioning::StateT::THETA, xbot::positioning::StateT::X);
        odometry.pose.covariance[31] = P(xbot::positioning::StateT::THETA, xbot::positioning::StateT::Y);
        odometry.twist.covariance[0] = P(xbot::positioning::StateT::VX, xbot::positioning::StateT::VX);
        odometry.twist.covariance[35] = P(xbot::positioning::StateT::VR, xbot::positioning::StateT::VR);
    }

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header = odometry.header;
    odom_trans.child_frame_id = odometry.child_frame_id;
    odom_trans.transform.translation.x = odometry.pose.pose.position.x;
    odom_trans.transform.translation.y = odometry.pose.pose.position.y;
    odom_trans.transform.translation.z = odometry.pose.pose.position.z;
    odom_trans.transform.rotation = odometry.pose.pose.orientation;

    static tf2_ros::TransformBroadcaster transform_broadcaster;
    transform_broadcaster.sendTransform(odom_trans);

    if (publish_debug) {
        auto state = core.getState();
        const auto &P = core.getCovariance();
        state_msg.x = state.x();
        state_msg.y = state.y();
        state_msg.theta = state.theta();
        state_msg.vx = state.vx();
        state_msg.vr = state.vr();

        state_msg.sigma_x = std::sqrt(P(xbot::positioning::StateT::X, xbot::positioning::StateT::X));
        state_msg.sigma_y = std::sqrt(P(xbot::positioning::StateT::Y, xbot::positioning::StateT::Y));
        state_msg.sigma_theta = std::sqrt(P(xbot::positioning::StateT::THETA, xbot::positioning::StateT::THETA));
        state_msg.position_innovation_x = core.last_position_innovation_x;
        state_msg.position_innovation_y = core.last_position_innovation_y;
        state_msg.position_nis = core.last_position_nis;

        kalman_state.publish(state_msg);
    }

    odometry_pub.publish(odometry);

    xb_absolute_pose_msg.header = odometry.header;
    xb_absolute_pose_msg.sensor_stamp = 0;
    xb_absolute_pose_msg.received_stamp = 0;
    xb_absolute_pose_msg.source = xbot_msgs::AbsolutePose::SOURCE_SENSOR_FUSION;
    xb_absolute_pose_msg.flags = xbot_msgs::AbsolutePose::FLAG_SENSOR_FUSION_DEAD_RECKONING;

    xb_absolute_pose_msg.orientation_valid = true;
    // TODO: send motion vector
    xb_absolute_pose_msg.motion_vector_valid = false;

    // Report the filter's own uncertainty rather than passing the GPS accuracy
    // through. mower_logic gates mowing on this, and a filter that has diverged
    // while the GPS is still happy needs to say so.
    {
        const auto &P = core.getCovariance();
        xb_absolute_pose_msg.position_accuracy = std::sqrt(
            P(xbot::positioning::StateT::X, xbot::positioning::StateT::X) +
            P(xbot::positioning::StateT::Y, xbot::positioning::StateT::Y));
        xb_absolute_pose_msg.orientation_accuracy =
            std::sqrt(P(xbot::positioning::StateT::THETA, xbot::positioning::StateT::THETA));
    }
    if (!has_gps) {
        xb_absolute_pose_msg.position_accuracy = 999;
    }
    if ((ros::Time::now() - last_gps_time).toSec() < 10.0) {
        xb_absolute_pose_msg.flags |= xbot_msgs::AbsolutePose::FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE;
    } else {
        // on GPS timeout, we set accuracy to 0.
        xb_absolute_pose_msg.position_accuracy = 999;
    }
    xb_absolute_pose_msg.pose = odometry.pose;
    xb_absolute_pose_msg.vehicle_heading = x.theta();
    xb_absolute_pose_msg.motion_heading = x.theta();

    xbot_absolute_pose_pub.publish(xb_absolute_pose_msg);


    last_imu = *msg;
}

void onWheelTicks(const xbot_msgs::WheelTick::ConstPtr &msg) {
    if (!has_ticks) {
        last_ticks = *msg;
        has_ticks = true;
        return;
    }
    double dt = (msg->stamp - last_ticks.stamp).toSec();
    if (dt <= 0.0) {
        last_ticks = *msg;
        return;
    }

    double d_wheel_l = (double) (msg->wheel_ticks_rl - last_ticks.wheel_ticks_rl) * (
                           1 / (double) msg->wheel_tick_factor);
    double d_wheel_r = (double) (msg->wheel_ticks_rr - last_ticks.wheel_ticks_rr) * (
                           1 / (double) msg->wheel_tick_factor);

    if (msg->wheel_direction_rl) {
        d_wheel_l *= -1.0;
    }
    if (msg->wheel_direction_rr) {
        d_wheel_r *= -1.0;
    }


    double d_ticks = (d_wheel_l + d_wheel_r) / 2.0;
    vx = d_ticks / dt;

    last_ticks = *msg;
}

void onTwistIn(const geometry_msgs::TwistStamped::ConstPtr &msg) {
    vx = msg->twist.linear.x;
}

bool setGpsState(xbot_positioning::GPSControlSrvRequest &req, xbot_positioning::GPSControlSrvResponse &res) {
    gps_enabled = req.gps_enabled;
    return true;
}

bool setPose(xbot_positioning::SetPoseSrvRequest &req, xbot_positioning::SetPoseSrvResponse &res) {
    tf2::Quaternion q;
    tf2::fromMsg(req.robot_pose.orientation, q);


    tf2::Matrix3x3 m(q);
    double unused1, unused2, yaw;

    m.getRPY(unused1, unused2, yaw);
    core.setState(req.robot_pose.position.x, req.robot_pose.position.y, yaw, 0, 0);
    return true;
}

void onPose(const xbot_msgs::AbsolutePose::ConstPtr &msg) {
  if (!gps_enabled) {
    return;
  }
    // TODO fuse with high covariance?
    if ((msg->flags & (xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FIXED)) == 0) {
        ROS_INFO_STREAM_THROTTLE(1, "Dropped GPS update, since it's not RTK Fixed");
        return;
    }

    if (!std::isfinite(msg->position_accuracy) || msg->position_accuracy > max_gps_accuracy) {
        ROS_INFO_STREAM_THROTTLE(
            1, "Dropped GPS update, since it's not accurate enough. Accuracy was: " << msg->position_accuracy <<
            ", limit is:" << max_gps_accuracy);
        return;
    }

    double time_since_last_gps = (ros::Time::now() - last_gps_time).toSec();
    if (time_since_last_gps > 5.0) {
        ROS_WARN_STREAM("Last GPS was " << time_since_last_gps << " seconds ago.");
        has_gps = false;
        valid_gps_samples = 0;
        gps_outlier_count = 0;
        last_gps = *msg;
        // we have GPS for next time
        last_gps_time = ros::Time::now();
        return;
    }

    tf2::Vector3 gps_pos(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    tf2::Vector3 last_gps_pos(last_gps.pose.pose.position.x, last_gps.pose.pose.position.y,
                              last_gps.pose.pose.position.z);

    double distance_to_last_gps = (last_gps_pos - gps_pos).length();

    if (distance_to_last_gps < 5.0) {
        // inlier, we treat it normally
        // store the gps as last
        last_gps = *msg;
        last_gps_time = ros::Time::now();

        gps_outlier_count = 0;
        valid_gps_samples++;

        if (!has_gps && valid_gps_samples > 10) {
            ROS_INFO_STREAM("GPS data now valid");
            ROS_INFO_STREAM(
                "First GPS data, moving kalman filter to " << msg->pose.pose.position.x << ", " << msg->pose.pose.
                position.y);
            // we don't even have gps yet, set odometry to first estimate
            core.updatePosition(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.001);

            has_gps = true;
        } else if (has_gps) {
            // Derive the measurement variance from what the receiver actually
            // reports, scaled (receivers are optimistic) and floored.
            const double sigma = std::max(static_cast<double>(msg->position_accuracy), gps_accuracy_floor) *
                                 gps_accuracy_scale;
            core.updatePosition(msg->pose.pose.position.x, msg->pose.pose.position.y, sigma * sigma);
            if (publish_debug) {
                auto m = core.om2.h(core.ekf.getState());
                geometry_msgs::Vector3 dbg;
                dbg.x = m.vx();
                dbg.y = m.vy();
                dbg_expected_motion_vector.publish(dbg);
            }
            // Only fuse the motion vector when the receiver says it is fresh.
            // Course over ground arrives less often than the position fix, and
            // re-fusing the same one drags heading toward where the robot was
            // pointing an epoch ago - worst exactly when turning.
            if (msg->motion_vector_valid &&
                std::sqrt(std::pow(msg->motion_vector.x, 2) + std::pow(msg->motion_vector.y, 2)) >= min_speed) {
                core.updateOrientation2(msg->motion_vector.x, msg->motion_vector.y, orientation_r);
            }
        }
    } else {
        ROS_WARN_STREAM("GPS outlier found. Distance was: " << distance_to_last_gps);
        gps_outlier_count++;
        // ~10 sec
        if (gps_outlier_count > 10) {
            ROS_ERROR_STREAM("too many outliers, assuming that the current gps value is valid.");
            // store the gps as last
            last_gps = *msg;
            last_gps_time = ros::Time::now();
            has_gps = false;

            valid_gps_samples = 0;
            gps_outlier_count = 0;
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "xbot_positioning");

    has_gps = false;
    gps_enabled = true;
    vx = 0.0;
    has_gyro = false;
    has_ticks = false;
    gyro_offset = 0;
    gyro_offset_samples = 0;

    valid_gps_samples = 0;
    gps_outlier_count = 0;

    antenna_offset_x = antenna_offset_y = 0;

    ros::NodeHandle n;
    ros::NodeHandle paramNh("~");

    ros::ServiceServer gps_service = n.advertiseService("xbot_positioning/set_gps_state", setGpsState);
    ros::ServiceServer pose_service = n.advertiseService("xbot_positioning/set_robot_pose", setPose);

    paramNh.param("skip_gyro_calibration", skip_gyro_calibration, false);
    paramNh.param("gyro_offset", gyro_offset, 0.0);
    // Course over ground is meaningless below a walking pace; 0.01 m/s fused
    // pure noise into the heading.
    paramNh.param("min_speed", min_speed, 0.15);
    paramNh.param("max_gps_accuracy", max_gps_accuracy, 0.1);
    paramNh.param("debug", publish_debug, false);
    paramNh.param("antenna_offset_x", antenna_offset_x, 0.0);
    paramNh.param("antenna_offset_y", antenna_offset_y, 0.0);

    // Process noise, variance per second.
    paramNh.param("q_xy", q_xy, 1e-4);
    paramNh.param("q_theta", q_theta, 1e-4);
    paramNh.param("q_vx", q_vx, 1e-2);
    paramNh.param("q_vr", q_vr, 1e-2);
    // Measurement noise.
    paramNh.param("r_vx", r_vx, 1e-3);
    paramNh.param("r_vr", r_vr, 4e-4);
    paramNh.param("gps_accuracy_scale", gps_accuracy_scale, 1.5);
    paramNh.param("gps_accuracy_floor", gps_accuracy_floor, 0.02);
    paramNh.param("orientation_r", orientation_r, 1e-2);
    paramNh.param("max_predict_dt", max_predict_dt, 0.5);

    core.setProcessNoise(q_xy, q_theta, q_vx, q_vr);
    core.setSpeedNoise(r_vx, r_vr);
    core.setAntennaOffset(antenna_offset_x, antenna_offset_y);

    ROS_INFO_STREAM("Antenna offset: " << antenna_offset_x << ", " << antenna_offset_y);

    if (gyro_offset != 0.0 && skip_gyro_calibration) {
        ROS_WARN_STREAM("Using gyro offset of: " << gyro_offset);
    }

    odometry_pub = paramNh.advertise<nav_msgs::Odometry>("odom_out", 50);
    xbot_absolute_pose_pub = paramNh.advertise<xbot_msgs::AbsolutePose>("xb_pose_out", 50);
    if (publish_debug) {
        dbg_expected_motion_vector = paramNh.advertise<geometry_msgs::Vector3>("debug_expected_motion_vector", 50);
        kalman_state = paramNh.advertise<xbot_positioning::KalmanState>("kalman_state", 50);
    }

    ros::Subscriber imu_sub = paramNh.subscribe("imu_in", 10, onImu);
    ros::Subscriber twist_sub = paramNh.subscribe("twist_in", 10, onTwistIn);
    ros::Subscriber pose_sub = paramNh.subscribe("xb_pose_in", 10, onPose);
    ros::Subscriber wheel_tick_sub = paramNh.subscribe("wheel_ticks_in", 10, onWheelTicks);

    ros::spin();
    return 0;
}
