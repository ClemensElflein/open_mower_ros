// Created by Clemens Elflein on 2/18/22, 5:37 PM.
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

// Include messages for mower control
#include "mower_msgs/Status.h"
#include "mower_msgs/MowerControlSrv.h"
#include "xbot_positioning/GPSControlSrv.h"
#include "mower_msgs/EmergencyStopSrv.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "mower_map/GetDockingPointSrv.h"

#include "dynamic_reconfigure/server.h"
#include "mower_simulation/MowerSimulationConfig.h"
#include "xbot_msgs/AbsolutePose.h"
#include "nav_msgs/Odometry.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "xbot_positioning/SetPoseSrv.h"

ros::Publisher status_pub;
ros::Publisher cmd_vel_pub;
ros::Publisher pose_pub;
ros::Publisher initial_pose_publisher;
ros::ServiceClient docking_point_client;

mower_msgs::Status fake_mow_status;
mower_simulation::MowerSimulationConfig config;
dynamic_reconfigure::Server<mower_simulation::MowerSimulationConfig> *reconfig_server;

geometry_msgs::Twist last_cmd_vel;

xbot_msgs::AbsolutePose pose{};
geometry_msgs::PoseWithCovarianceStamped initialPoseMsg;

bool has_dock = false;
double dockX = 0, dockY = 0;
bool is_docked = false;


bool setGpsState(xbot_positioning::GPSControlSrvRequest &req, xbot_positioning::GPSControlSrvResponse &res) {
    return true;
}

bool setMowEnabled(mower_msgs::MowerControlSrvRequest &req, mower_msgs::MowerControlSrvResponse &res) {
    config.mower_running = req.mow_enabled;
    reconfig_server->updateConfig(config);
    return true;
}

bool setEmergencyStop(mower_msgs::EmergencyStopSrvRequest &req, mower_msgs::EmergencyStopSrvResponse &res) {
    config.emergency_stop = req.emergency;
    reconfig_server->updateConfig(config);
    return true;
}
void fetchDock(const ros::TimerEvent &timer_event) {
    mower_map::GetDockingPointSrv get_docking_point_srv;
    geometry_msgs::PoseWithCovarianceStamped docking_pose_stamped;

    bool last_has_dock = has_dock;
    if(docking_point_client.call(get_docking_point_srv)) {
        has_dock = true;
        dockX = get_docking_point_srv.response.docking_pose.position.x;
        dockY = get_docking_point_srv.response.docking_pose.position.y;
    } else {
        has_dock = false;
    }

    if(last_has_dock != has_dock) {
        ROS_INFO_STREAM("map has a dock: " << (has_dock ? "YES" : "NO"));
    }

}

void publishStatus(const ros::TimerEvent &timer_event) {

    if (config.emergency_stop) {
        geometry_msgs::Twist emergency_twist;
        emergency_twist.linear.x = 0.0;
        emergency_twist.linear.y = 0.0;
        emergency_twist.linear.z = 0.0;
        emergency_twist.angular.x = 0.0;
        emergency_twist.angular.y = 0.0;
        emergency_twist.angular.z = 0.0;

        cmd_vel_pub.publish(emergency_twist);
    } else {
        cmd_vel_pub.publish(last_cmd_vel);
    }

    fake_mow_status.mow_esc_status.temperature_motor = config.temperature_mower;
    fake_mow_status.mow_esc_status.status = mower_msgs::ESCStatus::ESC_STATUS_OK;
    if (config.mower_error) {
        fake_mow_status.mow_esc_status.status = mower_msgs::ESCStatus ::ESC_STATUS_ERROR;;
    }
    if (config.mower_running) {
        fake_mow_status.mow_esc_status.status = mower_msgs::ESCStatus::ESC_STATUS_RUNNING;
    }

    fake_mow_status.v_battery = config.battery_voltage;

    bool last_is_docked = is_docked;
    is_docked = has_dock && sqrt(pow(pose.pose.pose.position.x - dockX, 2) + pow(pose.pose.pose.position.y - dockY, 2)) < 0.25;

    if(last_is_docked != is_docked) {
        ROS_INFO_STREAM("simulation is inside dock: " << (is_docked ? "YES" : "NO"));
    }

    if(is_docked) {
        // "docked", simulate charge voltage.
        fake_mow_status.v_charge = 29.4;
    } else {
        // Not docked, use manual charge flag for charging status
        fake_mow_status.v_charge = config.is_charging ? 29.5 : 0.0;
    }
    if (config.wheels_stalled) {
        fake_mow_status.left_esc_status.status = mower_msgs::ESCStatus ::ESC_STATUS_STALLED;
        fake_mow_status.right_esc_status.status = mower_msgs::ESCStatus ::ESC_STATUS_STALLED;
    } else {
        fake_mow_status.left_esc_status.status = mower_msgs::ESCStatus::ESC_STATUS_OK;
        fake_mow_status.right_esc_status.status = mower_msgs::ESCStatus::ESC_STATUS_OK;
    }
    fake_mow_status.emergency = config.emergency_stop;

    status_pub.publish(fake_mow_status);
}

void reconfigureCB(mower_simulation::MowerSimulationConfig &c, uint32_t level) {
    config = c;
}

void velReceived(const geometry_msgs::Twist::ConstPtr &msg) {
    last_cmd_vel = *msg;
}
void odomReceived(const nav_msgs::Odometry::ConstPtr &msg) {

    pose.header = msg->header;
    pose.pose = msg->pose;
    pose.orientation_valid = true;
    pose.orientation_accuracy = 0.1;
    pose.position_accuracy = 0.02;
    pose.source = xbot_msgs::AbsolutePose::SOURCE_SENSOR_FUSION;
    pose.flags = xbot_msgs::AbsolutePose::FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE;

    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.pose.orientation, q);


    tf2::Matrix3x3 m(q);
    double unused1, unused2, yaw;

    m.getRPY(unused1, unused2, yaw);

    pose.vehicle_heading = yaw;
    pose.motion_heading = yaw;
    pose.motion_vector_valid = false;

    pose_pub.publish(pose);

}

bool setPose(xbot_positioning::SetPoseSrvRequest &req, xbot_positioning::SetPoseSrvResponse &res) {
    double yaw;
    {
        tf2::Quaternion q;
        tf2::fromMsg(req.robot_pose.orientation, q);


        tf2::Matrix3x3 m(q);
        double unused1, unused2;

        m.getRPY(unused1, unused2, yaw);
    }

    tf2::Quaternion q(0.0, 0.0, yaw);


    initialPoseMsg.header.stamp = ros::Time::now();
    initialPoseMsg.header.frame_id = "base_link";
    initialPoseMsg.header.seq++;
    initialPoseMsg.pose.pose.position.x = req.robot_pose.position.x;
    initialPoseMsg.pose.pose.position.y = req.robot_pose.position.y;
    initialPoseMsg.pose.pose.position.z = 0;
    initialPoseMsg.pose.pose.orientation = tf2::toMsg(q);

    initial_pose_publisher.publish(initialPoseMsg);


    return true;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "mower_simulation");


    ros::NodeHandle n;
    ros::NodeHandle paramNh("~");




    reconfig_server = new dynamic_reconfigure::Server<mower_simulation::MowerSimulationConfig>(paramNh);
    reconfig_server->setCallback(reconfigureCB);

    docking_point_client = n.serviceClient<mower_map::GetDockingPointSrv>(
            "mower_map_service/get_docking_point");

    initial_pose_publisher = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);

    /*
    This introduces more issues than it solves..

    ROS_INFO("Waiting for map service");
    if (!docking_point_client.waitForExistence(ros::Duration(60.0, 0.0))) {
        ROS_ERROR("No map client found; we can't set robot's initial pose.");
    } else {
        mower_map::GetDockingPointSrv get_docking_point_srv;
        geometry_msgs::PoseWithCovarianceStamped docking_pose_stamped;

        docking_point_client.call(get_docking_point_srv);
        docking_pose_stamped.pose.pose = get_docking_point_srv.response.docking_pose;
        docking_pose_stamped.header.frame_id = "map";
        docking_pose_stamped.header.stamp = ros::Time::now();
        initial_pose_publisher.publish(docking_pose_stamped);
    }
    */

    fake_mow_status.mower_status = mower_msgs::Status::MOWER_STATUS_OK;
    fake_mow_status.v_charge = 0.0;
    fake_mow_status.v_battery = 29.0;
    fake_mow_status.stamp = ros::Time::now();
    fake_mow_status.left_esc_status.status = fake_mow_status.right_esc_status.status = fake_mow_status.mow_esc_status.status = mower_msgs::ESCStatus::ESC_STATUS_OK;
    fake_mow_status.mow_esc_status.temperature_motor = 50;
    fake_mow_status.emergency = true;
    config.emergency_stop = true;

    status_pub = n.advertise<mower_msgs::Status>("mower/status", 1);
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_out", 1);
    pose_pub = n.advertise<xbot_msgs::AbsolutePose>("/xbot_positioning/xb_pose", 1);

    ros::Subscriber cmd_vel_sub = n.subscribe("/cmd_vel", 0, velReceived, ros::TransportHints().tcpNoDelay(true));
    ros::Subscriber odom_sub = n.subscribe("/mower/odom", 0, odomReceived, ros::TransportHints().tcpNoDelay(true));
    ros::ServiceServer mow_service = n.advertiseService("mower_service/mow_enabled", setMowEnabled);
    ros::ServiceServer gps_service = n.advertiseService("xbot_positioning/set_gps_state", setGpsState);
    ros::ServiceServer emergency_service = n.advertiseService("mower_service/emergency", setEmergencyStop);
    ros::ServiceServer pose_service = n.advertiseService("xbot_positioning/set_robot_pose", setPose);

    ros::Timer publish_timer = n.createTimer(ros::Duration(0.02), publishStatus);
    ros::Timer update_dock_timer = n.createTimer(ros::Duration(1.0), fetchDock);

    ros::spin();
    delete (reconfig_server);
    return 0;
}
