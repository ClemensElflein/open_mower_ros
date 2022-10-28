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

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include "xbot_msgs/AbsolutePose.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"

ros::Publisher odometry_pub;
ros::Publisher imu_pub;

sensor_msgs::Imu last_imu;
bool has_gyro;
ros::Time gyro_calibration_start;
double gyro_offset;
int gyro_offset_samples;


geometry_msgs::PoseWithCovarianceStamped odometry;
geometry_msgs::TwistWithCovarianceStamped odometry_imu;

void onImu(const sensor_msgs::Imu::ConstPtr &msg) {
    if(!has_gyro) {
        if(gyro_offset_samples == 0) {
            ROS_INFO_STREAM("Started gyro calibration");
            gyro_calibration_start = msg->header.stamp;
            gyro_offset = 0;
        }
        gyro_offset += msg->angular_velocity.z;
        gyro_offset_samples++;
        if((msg->header.stamp - gyro_calibration_start).toSec() < 10) {
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
    last_imu = *msg;
    odometry_imu.header.stamp = msg->header.stamp;
    odometry_imu.header.frame_id = "base_link";
    odometry_imu.header.seq++;
    odometry_imu.twist.twist.angular.z = 1.0;//msg->angular_velocity.z - gyro_offset;

    odometry_imu.twist.covariance = {
            -1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, -1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, -1, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, -1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, -1, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0001
    };

    imu_pub.publish(odometry_imu);
}

void onPose(const xbot_msgs::AbsolutePose::ConstPtr &msg) {
    if(!has_gyro)
        return;
    odometry.header.seq++;
    odometry.header.frame_id = "map";
    odometry.header.stamp = ros::Time::now();




    // Set position from the pose
    odometry.pose.pose.position = msg->pose.pose.position;
    odometry.pose.covariance = {
            msg->pose.covariance[0], 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, msg->pose.covariance[7], 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, msg->pose.covariance[14], 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, -1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, -1.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, -1.0
    };
    // Set the orientation from the movement vector
    tf2::Quaternion q_mag(0.0, 0.0, msg->motion_heading);
    odometry.pose.pose.orientation = tf2::toMsg(q_mag);

//    odometry.pose.covariance[35] = msg->orientation_accuracy*pow(abs(last_imu.angular_velocity.z-gyro_offset)+1.0, 2.0);

    odometry_pub.publish(odometry);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "ekf_odometry");

    has_gyro = false;
    gyro_offset = 0;
    gyro_offset_samples = 0;

    ros::NodeHandle n;
    ros::NodeHandle paramNh("~");

    odometry_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("ekf_odometry/odom", 50);
    imu_pub = n.advertise<geometry_msgs::TwistWithCovarianceStamped>("ekf_odometry/imu", 50);

    ros::Subscriber imu_sub = n.subscribe("imu/data_raw", 10, onImu);
    ros::Subscriber pose_sub = n.subscribe("driver_gps_node/xb_pose", 10, onPose);

    ros::spin();
    return 0;
}
