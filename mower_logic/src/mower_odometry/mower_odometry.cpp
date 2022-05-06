// Created by Clemens Elflein on 3/28/22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//
// This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
//
// Feel free to use the design in your private/educational projects, but don't try to sell the design or products based on it without getting my consent first.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
//

#include "ros/ros.h"

#include <mower_msgs/Status.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include "ublox_msgs/NavRELPOSNED9.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include "mower_logic/MowerOdometryConfig.h"
#include <dynamic_reconfigure/server.h>
#include "mower_msgs/GPSControlSrv.h"


ros::Publisher odometry_pub;

ublox_msgs::NavRELPOSNED9 last_gps;

sensor_msgs::Imu lastImu;
mower_logic::MowerOdometryConfig config;

int gps_outlier_count = 0;
int valid_gps_samples = 0;

bool hasImuMessage = false;
bool gpsOdometryValid = false;
ros::Time last_gps_odometry_time(0.0);
bool gpsEnabled = true;

// Odometry
bool firstData = true;
mower_msgs::Status last_status;

// inputs here
double d_wheel_r, d_wheel_l, dt = 1.0;

// outputs here
double x = 0, y = 0, vx = 0.0, r = 0.0, vy = 0.0, vr = 0.0;
geometry_msgs::Quaternion orientation_result;


// (ticks / revolution) / (m / revolution)
#define TICKS_PER_M (993.0 / (0.19*M_PI))


tf2_ros::Buffer tfBuffer;


double getGPSY(ublox_msgs::NavRELPOSNED9 &msg) {
    return ((double) msg.relPosN * 0.01) + ((double) msg.relPosHPN * 0.0001);
}

double getGPSX(ublox_msgs::NavRELPOSNED9 &msg) {
    return ((double) msg.relPosE * 0.01) + ((double) msg.relPosHPE * 0.0001);
}

double getGPSZ(ublox_msgs::NavRELPOSNED9 &msg) {
    // For now, we assume a plane
    return 0.0;
}


void publishOdometry() {
    static tf2_ros::TransformBroadcaster transform_broadcaster;


    geometry_msgs::TransformStamped odom_trans;
    ros::Time current_time = ros::Time::now();
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "map";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    // TODO: Add logic for 3d odometry
    odom_trans.transform.translation.z = 0;
    odom_trans.transform.rotation = orientation_result;


    transform_broadcaster.sendTransform(odom_trans);

    // next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "map";

    // set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;

    odom.pose.pose.orientation = odom_trans.transform.rotation;

    // set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vr;


    odom.pose.covariance = {
            10000.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 10000.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 10000.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 10000.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.00001
    };

    if (gpsOdometryValid && gpsEnabled && ros::Time::now() - last_gps_odometry_time < ros::Duration(5.0)) {
        odom.pose.covariance[0] = last_gps.accLength / 10000.0;
        odom.pose.covariance[7] = last_gps.accLength / 10000.0;
    }

    odom.twist.covariance = {
            0.000001, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.000001, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 10000.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 10000.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.000001
    };




    // publish the message
    odometry_pub.publish(odom);
}

void imuReceived(const sensor_msgs::Imu::ConstPtr &msg) {
    lastImu = *msg;
    hasImuMessage = true;
}


void gpsPositionReceived(const ublox_msgs::NavRELPOSNED9::ConstPtr &msg) {
    ublox_msgs::NavRELPOSNED9 gps = *msg;
    double gps_accuracy_m = (double) gps.accLength / 10000.0f;

    bool gnssFixOK = (msg->flags & 0b0000001);
    bool diffSoln = (msg->flags & 0b0000010) >> 1;
    bool relPosValid = (msg->flags & 0b0000100) >> 2;
    auto carrSoln = (uint8_t) ((msg->flags & 0b0011000) >> 3);
    bool refPosMiss = (msg->flags & 0b0000100) >> 6;
    bool refObsMiss = (msg->flags & 0b0000100) >> 7;

    if (!gnssFixOK || !diffSoln || !relPosValid || carrSoln != 2) {
        ROS_INFO_STREAM_THROTTLE(1,"Dropped at least one GPS update due to flags.\r\nFlags:\r\n" <<
                                                                          "accuracy:" << gps_accuracy_m << "\r\n" <<
                                                                          "gnssFixOK:" << gnssFixOK << "\r\n" <<
                                                                          "diffSoln:" << diffSoln << "\r\n" <<
                                                                          "relPosValid:" << relPosValid << "\r\n" <<
                                                                          "carrSoln:" << (int) carrSoln << "\r\n" <<
                                                                          "refPosMiss:" << refPosMiss << "\r\n" <<
                                                                          "refObsMiss:" << refObsMiss << "\r\n"
        );
        return;
    }

    if (gps_accuracy_m > 0.05) {
        ROS_INFO_STREAM("dropping gps with accuracy: " << gps_accuracy_m << "m");
        return;
    }

    if (!gpsEnabled) {
        gpsOdometryValid = false;
        valid_gps_samples = 0;
        return;
    }

    double time_since_last_gps = (ros::Time::now() - last_gps_odometry_time).toSec();
    if (time_since_last_gps > 5.0) {
        ROS_WARN_STREAM("Last GPS was " << time_since_last_gps << " seconds ago.");
        gpsOdometryValid = false;
        valid_gps_samples = 0;
        gps_outlier_count = 0;
        last_gps = gps;
        last_gps_odometry_time = ros::Time::now();
        return;
    }


    tf2::Vector3 last_gps_pos(
            getGPSX(last_gps), getGPSY(last_gps), getGPSZ(last_gps)
    );
    tf2::Vector3 gps_pos(
            getGPSX(gps), getGPSY(gps), getGPSZ(gps)
    );

//    ROS_INFO_STREAM("GOT GPS: " << gps_pos.x() << ", " << gps_pos.y());


    double distance_to_last_gps = (last_gps_pos - gps_pos).length();

    if (distance_to_last_gps < 5.0) {
        // inlier, we treat it normally

        // calculate current base_link position from orientation and distance parameter

        double base_link_x = gps_pos.x() - config.gps_antenna_offset * cos(r);
        double base_link_y = gps_pos.y() - config.gps_antenna_offset * sin(r);


        // store the gps as last
        last_gps = gps;
        last_gps_odometry_time = ros::Time::now();

        gps_outlier_count = 0;
        valid_gps_samples++;
        if (!gpsOdometryValid && valid_gps_samples > 10) {
            ROS_INFO_STREAM("GPS data now valid");
            ROS_INFO_STREAM("First GPS data, moving odometry to " << x << ", " << y);
            // we don't even have gps yet, set odometry to first estimate
            x = base_link_x;
            y = base_link_y;
            gpsOdometryValid = true;
        } else if (gpsOdometryValid) {
            // gps was valid before, we apply the filter
            x = x * (1.0 - config.gps_filter_factor) + config.gps_filter_factor * base_link_x;
            y = y * (1.0 - config.gps_filter_factor) + config.gps_filter_factor * base_link_y;
        }
    } else {
        ROS_WARN_STREAM("GPS outlier found. Distance was: " << distance_to_last_gps);
        gps_outlier_count++;
        // ~10 sec
        if (gps_outlier_count > 10) {
            ROS_ERROR_STREAM("too many outliers, assuming that the current gps value is valid.");
            last_gps = gps;
            last_gps_odometry_time = ros::Time::now();

            gpsOdometryValid = false;
            valid_gps_samples = 0;
            gps_outlier_count = 0;
        }
    }
}


bool statusReceivedOrientation(const mower_msgs::Status::ConstPtr &msg) {

    if (!hasImuMessage) {
        ROS_INFO("waiting for imu message");
        return false;
    }




//    geometry_msgs::TransformStamped imu_to_base_link = tfBuffer.lookupTransform(lastImu.header.frame_id, "base_link",
//                                                                                lastImu.header.stamp);



/*
    double r_gyro = r - lastImu.gz * dt;
    while (r_gyro < 0) {
        r_gyro += 2.0 * M_PI;
    }
    r_gyro = fmod(r_gyro, 2.0 * M_PI);
    tf2::Quaternion q_gyro(0.0, 0.0, r_gyro);



//    geometry_msgs::Quaternion base_link_pose;
//    tf2::doTransform(imu_pose, base_link_pose, imu_to_base_link);


    double yaw = atan2(lastImu.my - (config.magnetic_offset_y),
                       lastImu.mx - (config.magnetic_offset_x));

    yaw += config.imu_offset * (M_PI / 180.0);
    yaw = fmod(yaw + (M_PI_2), 2.0 * M_PI);
    while (yaw < 0) {
        yaw += M_PI * 2.0;
    }


    tf2::Quaternion q_mag(0.0, 0.0, yaw);


//    r = yaw;
//    dr = lastImu.angular_velocity.z;

*/
    double d_ticks = (d_wheel_l + d_wheel_r) / 2.0;
    /*

//    dr = yaw - r;
//    dr = fmod(dr, 2.0 * M_PI);


    tf2::Quaternion q = q_gyro.slerp(q_mag, config.magnetic_filter_factor);
*/

    orientation_result = lastImu.orientation;
    tf2::Quaternion q;
    tf2::fromMsg(orientation_result, q);


    tf2::Matrix3x3 m(q);
    double unused1, unused2;

    m.getRPY(unused1, unused2, r);


    x += d_ticks * cos(r);
    y += d_ticks * sin(r);

    vy = 0;
    vx = d_ticks / dt;
    vr = lastImu.angular_velocity.z;


    return true;
}


void statusReceived(const mower_msgs::Status::ConstPtr &msg) {

    // we need the differences, so initialize in the first run
    if (firstData) {

        last_status = *msg;
        firstData = false;

        return;
    }

    dt = (msg->stamp - last_status.stamp).toSec();


    d_wheel_l = (int32_t) (msg->left_esc_status.tacho - last_status.left_esc_status.tacho) / TICKS_PER_M;
    d_wheel_r = -(int32_t) (msg->right_esc_status.tacho - last_status.right_esc_status.tacho) / TICKS_PER_M;


//    ROS_INFO_STREAM("d_wheel_l = " << d_wheel_l << ", d_wheel_r = " << d_wheel_r);

    bool success = statusReceivedOrientation(msg);

    last_status = *msg;

    if (success) {
        publishOdometry();
    }
}

void reconfigureCB(mower_logic::MowerOdometryConfig &c, uint32_t level) {
    ROS_INFO_STREAM("Setting new Mower Odom Config");
    config = c;
}

bool setGpsState(mower_msgs::GPSControlSrvRequest &req, mower_msgs::GPSControlSrvResponse &res) {
    gpsEnabled = req.gps_enabled;
    gpsOdometryValid = false;
    ROS_WARN_STREAM("Setting GPS enabled to: " << gpsEnabled);
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mower_odometry");


    ros::NodeHandle n;
    ros::NodeHandle paramNh("~");

    ros::ServiceServer gps_service = n.advertiseService("mower_service/set_gps_state", setGpsState);

    dynamic_reconfigure::Server<mower_logic::MowerOdometryConfig> reconfig_server(paramNh);
    reconfig_server.setCallback(reconfigureCB);


    tf2_ros::TransformListener tfListener(tfBuffer);


    odometry_pub = n.advertise<nav_msgs::Odometry>("mower/odom", 50);

    firstData = true;
    gps_outlier_count = 0;
    gpsOdometryValid = false;

    ros::Subscriber status_sub = n.subscribe("mower/status", 100, statusReceived);
    ros::Subscriber imu_sub = n.subscribe("mower/imu", 100, imuReceived);
    ros::Subscriber gps_sub = n.subscribe("ublox/navrelposned", 100, gpsPositionReceived);

    ros::spin();
    return 0;
}
