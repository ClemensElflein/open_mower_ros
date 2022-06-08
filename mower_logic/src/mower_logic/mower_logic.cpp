// Created by Clemens Elflein on 2/21/22.
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
#include "slic3r_coverage_planner/PlanPath.h"
#include "mower_map/GetMowingAreaSrv.h"
#include "mower_map/GetDockingPointSrv.h"
#include "mower_map/SetDockingPointSrv.h"
#include "mower_map/ClearNavPointSrv.h"
#include "mower_map/SetNavPointSrv.h"
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Transform.h>
#include "mower_msgs/GPSControlSrv.h"
#include "mbf_msgs/ExePathAction.h"
#include "mbf_msgs/MoveBaseAction.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "mower_msgs/Status.h"
#include "actionlib/client/simple_client_goal_state.h"
#include "mower_msgs/MowerControlSrv.h"
#include "mower_msgs/EmergencyStopSrv.h"
#include "ftc_local_planner/PlannerGetProgress.h"
#include <dynamic_reconfigure/server.h>
#include "mower_logic/MowerLogicConfig.h"
#include "behaviors/Behavior.h"
#include "behaviors/IdleBehavior.h"
#include "behaviors/AreaRecordingBehavior.h"
#include "mower_msgs/HighLevelControlSrv.h"
#include "std_msgs/String.h"

ros::ServiceClient pathClient, mapClient, dockingPointClient, gpsClient, mowClient, emergencyClient, pathProgressClient, setNavPointClient, clearNavPointClient;

ros::NodeHandle *n;
ros::NodeHandle *paramNh;

dynamic_reconfigure::Server<mower_logic::MowerLogicConfig> *reconfigServer;
actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction> *mbfClient;
actionlib::SimpleActionClient<mbf_msgs::ExePathAction> *mbfClientExePath;

ros::Publisher cmd_vel_pub;
mower_logic::MowerLogicConfig last_config;


// store some values for safety checks
ros::Time odom_time(0.0);
nav_msgs::Odometry last_odom;
ros::Time status_time(0.0);
mower_msgs::Status last_status;

ros::Time last_good_gps(0.0);


bool mowerEnabled = false;

// true, if bot needs to dock or needs to stay docked.
// This can be set to true by anyone but will only be set to false by the idle state on exit
bool mowingPaused = false;

Behavior *currentBehavior = &IdleBehavior::INSTANCE;

void odomReceived(const nav_msgs::Odometry::ConstPtr &msg) {
    last_odom = *msg;
    odom_time = ros::Time::now();
}

void statusReceived(const mower_msgs::Status::ConstPtr &msg) {
    last_status = *msg;
    status_time = ros::Time::now();
}

void pauseExecution() {
    if (currentBehavior != nullptr) {
        currentBehavior->pause();
    }
    if (!mowingPaused) {
        ROS_INFO_STREAM("Requesting Pause");
    }
    mowingPaused = true;
}

bool setGPS(bool enabled) {
    mower_msgs::GPSControlSrv gps_srv;
    gps_srv.request.gps_enabled = enabled;
    // TODO check result
    gpsClient.call(gps_srv);
    return true;
}


bool setMowerEnabled(bool enabled) {

    if(enabled == mowerEnabled) {
        return true;
    }

    mowerEnabled = enabled;

    if (!last_config.enable_mower && enabled) {
        ROS_INFO_STREAM("Mower should be enabled but is disabled in the config.");
        enabled = false;
    }

    mower_msgs::MowerControlSrv mow_srv;
    mow_srv.request.mow_enabled = enabled;
    mowClient.call(mow_srv);

/*    if (enabled) {
        ROS_INFO_STREAM("enabled mower, waiting for it to speed up");

        // TODO timeout and error
        ros::Time started = ros::Time::now();
        while (true) {
            if (status_time > started) {
                // we have a current status message, wait for mower to speed up
                bool mower_running = (last_status.speed_mow_status & 0b10);
                if (mower_running) {
                    ROS_INFO_STREAM("mower motor started");
                    return true;
                }
            }
            if (ros::Time::now() - started > ros::Duration(25.0)) {
                // mower was not able to start
                ROS_ERROR_STREAM("error starting mower motor...");
                setMowerEnabled(false);
                return false;
            }
        }
    }*/

    return true;
}


void stop() {
    geometry_msgs::Twist stop;
    stop.angular.z = 0;
    stop.linear.x = 0;
    cmd_vel_pub.publish(stop);

    setMowerEnabled(false);

    // to be sure, call it again.
    mower_msgs::MowerControlSrv mow_srv;
    mow_srv.request.mow_enabled = false;
    mowClient.call(mow_srv);
}
void setEmergencyMode(bool emergency) {
    mower_msgs::EmergencyStopSrv emergencyStop;
    emergencyStop.request.emergency = emergency;
    emergencyClient.call(emergencyStop);
}

void checkSafety(const ros::TimerEvent &timer_event) {
    // check if odometry is current. if not, we have a problem -> emergency
    if (ros::Time::now() - odom_time > ros::Duration(1.0)) {
        setEmergencyMode(true);
        ROS_ERROR_STREAM("EMERGENCY: odometry values stopped. dt was: " << (ros::Time::now() - odom_time));
        return;
    }

    // check if status is current. if not, we have a problem since it contains wheel ticks and so on -> emergency
    if (ros::Time::now() - status_time > ros::Duration(0.5)) {
        setEmergencyMode(true);
        ROS_ERROR_STREAM("EMERGENCY: status values stopped. dt was: " << (ros::Time::now() - status_time));
        return;
    }

    if (last_status.right_esc_status.status <= mower_msgs::ESCStatus::ESC_STATUS_ERROR || last_status.left_esc_status.status <= mower_msgs::ESCStatus ::ESC_STATUS_ERROR) {
        setEmergencyMode(true);
        ROS_ERROR_STREAM(
                "EMERGENCY: at least one motor control errored. errors left: " << (last_status.left_esc_status)
                                                                               << ", status right: "
                                                                               << last_status.right_esc_status);
        return;
    }
    bool gpsGood = last_odom.pose.covariance[0] < 0.05 && last_odom.pose.covariance[0] > 0;
    if (gpsGood || last_config.ignore_gps_errors) {
        last_good_gps = ros::Time::now();
    }

    if (currentBehavior != nullptr && currentBehavior->needs_gps() && ros::Time::now() - last_good_gps > ros::Duration(last_config.gps_timeout)) {
        ROS_WARN_STREAM("gps lost, stopping");
        setEmergencyMode(true);
        return;
    }

    // check if we need the mower.
    setMowerEnabled(currentBehavior != nullptr && currentBehavior->mower_enabled());

    // we are in non emergency, check if we should pause. This could be empty battery, rain or hot mower motor etc.
    bool dockingNeeded = false;

    if (last_status.v_battery < last_config.battery_empty_voltage || last_status.mow_esc_status.temperature_motor >= last_config.motor_hot_temperature ||
        last_config.manual_pause_mowing) {
        dockingNeeded = true;
    }

    if (dockingNeeded) {
        pauseExecution();
    }
}

void reconfigureCB(mower_logic::MowerLogicConfig &c, uint32_t level) {
    ROS_INFO_STREAM("Setting new Mow Area Config");
    last_config = c;
}

bool highLevelCommand(mower_msgs::HighLevelControlSrvRequest &req, mower_msgs::HighLevelControlSrvResponse &res) {
    switch(req.command) {
        case mower_msgs::HighLevelControlSrvRequest::COMMAND_HOME:
            if(currentBehavior) {
                currentBehavior->command_home();
            }
            break;
        case mower_msgs::HighLevelControlSrvRequest::COMMAND_START:
            if(currentBehavior) {
                currentBehavior->command_start();
            }
            break;
            case mower_msgs::HighLevelControlSrvRequest::COMMAND_S1:
            if(currentBehavior) {
                currentBehavior->command_s1();
            }
            break;
            case mower_msgs::HighLevelControlSrvRequest::COMMAND_S2:
            if(currentBehavior) {
                currentBehavior->command_s2();
            }
            break;

    }
    return true;
}

void joyVelReceived(const geometry_msgs::Twist::ConstPtr &joy_vel) {
    if(currentBehavior && currentBehavior->redirect_joystick()) {
        cmd_vel_pub.publish(joy_vel);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mower_logic");

    n = new ros::NodeHandle();
    paramNh = new ros::NodeHandle("~");

    boost::recursive_mutex mutex;

    reconfigServer = new dynamic_reconfigure::Server<mower_logic::MowerLogicConfig>(mutex, *paramNh);
    reconfigServer->setCallback(reconfigureCB);

    cmd_vel_pub = n->advertise<geometry_msgs::Twist>("/logic_vel", 1);

    ros::Publisher path_pub;
    ros::Publisher current_state_pub;


    path_pub = n->advertise<nav_msgs::Path>("mower_logic/mowing_path", 100, true);
    current_state_pub = n->advertise<std_msgs::String>("mower_logic/current_state", 100, true);

    pathClient = n->serviceClient<slic3r_coverage_planner::PlanPath>(
            "slic3r_coverage_planner/plan_path");
    mapClient = n->serviceClient<mower_map::GetMowingAreaSrv>(
            "mower_map_service/get_mowing_area");
    gpsClient = n->serviceClient<mower_msgs::GPSControlSrv>(
            "mower_service/set_gps_state");
    mowClient = n->serviceClient<mower_msgs::MowerControlSrv>(
            "mower_service/mow_enabled");
    emergencyClient = n->serviceClient<mower_msgs::EmergencyStopSrv>(
            "mower_service/emergency");

    dockingPointClient = n->serviceClient<mower_map::GetDockingPointSrv>(
            "mower_map_service/get_docking_point");

    pathProgressClient = n->serviceClient<ftc_local_planner::PlannerGetProgress>(
            "/move_base_flex/FTCPlanner/planner_get_progress");

    setNavPointClient = n->serviceClient<mower_map::SetNavPointSrv>(
            "mower_map_service/set_nav_point");
    clearNavPointClient = n->serviceClient<mower_map::ClearNavPointSrv>(
            "mower_map_service/clear_nav_point");



    mbfClient = new actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>("/move_base_flex/move_base");
    mbfClientExePath = new actionlib::SimpleActionClient<mbf_msgs::ExePathAction>("/move_base_flex/exe_path");


    ros::Subscriber status_sub = n->subscribe("/mower/status", 0, statusReceived,
                                             ros::TransportHints().tcpNoDelay(true));
    ros::Subscriber odom_sub = n->subscribe("/mower/odom", 0, odomReceived, ros::TransportHints().tcpNoDelay(true));
    ros::Subscriber joy_cmd = n->subscribe("/joy_vel", 0, joyVelReceived, ros::TransportHints().tcpNoDelay(true));

    ros::ServiceServer high_level_control_srv = n->advertiseService("mower_service/high_level_control", highLevelCommand);


    ros::AsyncSpinner asyncSpinner(1);
    asyncSpinner.start();

    ROS_INFO("Waiting for a status message");
    while (status_time == ros::Time(0.0)) {
        if (!ros::ok()) {
            delete (reconfigServer);
            delete (mbfClient);
            delete (mbfClientExePath);
            return 1;
        }
    }
    ROS_INFO("Waiting for a odometry message");
    while (odom_time == ros::Time(0.0)) {
        if (!ros::ok()) {
            delete (reconfigServer);
            delete (mbfClient);
            delete (mbfClientExePath);
            return 1;
        }
    }


    ROS_INFO("Waiting for emergency service");
    if (!emergencyClient.waitForExistence(ros::Duration(60.0, 0.0))) {
        ROS_ERROR("Emergency server not found.");
        delete (reconfigServer);
        delete (mbfClient);
        delete (mbfClientExePath);

        return 1;
    }


    ROS_INFO("Waiting for path server");
    if (!pathClient.waitForExistence(ros::Duration(60.0, 0.0))) {
        ROS_ERROR("Path service not found.");
        delete (reconfigServer);
        delete (mbfClient);
        delete (mbfClientExePath);


        return 1;
    }
    ROS_INFO("Waiting for gps service");
    if (!gpsClient.waitForExistence(ros::Duration(60.0, 0.0))) {
        ROS_ERROR("GPS service not found.");
        delete (reconfigServer);
        delete (mbfClient);
        delete (mbfClientExePath);


        return 1;
    }
    ROS_INFO("Waiting for mower service");
    if (!mowClient.waitForExistence(ros::Duration(60.0, 0.0))) {
        ROS_ERROR("Mower service not found.");
        delete (reconfigServer);
        delete (mbfClient);
        delete (mbfClientExePath);


        return 1;
    }


    ROS_INFO("Waiting for map server");
    if (!mapClient.waitForExistence(ros::Duration(60.0, 0.0))) {
        ROS_ERROR("Map server service not found.");
        delete (reconfigServer);
        delete (mbfClient);
        delete (mbfClientExePath);
        return 2;
    }
    ROS_INFO("Waiting for docking point server");
    if (!dockingPointClient.waitForExistence(ros::Duration(60.0, 0.0))) {
        ROS_ERROR("Docking server service not found.");
        delete (reconfigServer);
        delete (mbfClient);
        delete (mbfClientExePath);
        return 2;
    }
    ROS_INFO("Waiting for nav point server");
    if (!setNavPointClient.waitForExistence(ros::Duration(60.0, 0.0))) {
        ROS_ERROR("Set Nav Point server service not found.");
        delete (reconfigServer);
        delete (mbfClient);
        delete (mbfClientExePath);
        return 2;
    }
    ROS_INFO("Waiting for clear nav point server");
    if (!clearNavPointClient.waitForExistence(ros::Duration(60.0, 0.0))) {
        ROS_ERROR("Clear Nav Point server service not found.");
        delete (reconfigServer);
        delete (mbfClient);
        delete (mbfClientExePath);
        return 2;
    }

    ROS_INFO("Waiting for move base flex");
    if (!mbfClient->waitForServer(ros::Duration(60.0, 0.0))) {
        ROS_ERROR("Move base flex not found.");
        delete (reconfigServer);
        delete (mbfClient);
        delete (mbfClientExePath);
        return 3;
    }

    ROS_INFO("Waiting for mowing path progress server");
    if (!pathProgressClient.waitForExistence(ros::Duration(60.0, 0.0))) {
        ROS_ERROR("FTCLocalPlanner progress server not found.");
        delete (reconfigServer);
        delete (mbfClient);
        delete (mbfClientExePath);
        return 3;
    }

    ROS_INFO("Got all servers, we can mow");



    // release emergency if it was set
    setEmergencyMode(false);

    ros::Timer safety_timer = n->createTimer(ros::Duration(0.5), checkSafety);


    // Behavior execution loop

    while (ros::ok()) {
        if (currentBehavior != nullptr) {
            std_msgs::String state_name;
            state_name.data = currentBehavior->state_name();
            current_state_pub.publish(state_name);
            currentBehavior->start(last_config);
            if (mowingPaused) {
                currentBehavior->pause();
            }
            Behavior *newBehavior = currentBehavior->execute();
            currentBehavior->exit();
            currentBehavior = newBehavior;
        } else {
            std_msgs::String state_name;
            state_name.data = "NULL";
            current_state_pub.publish(state_name);
            // we have no defined behavior, set emergency
            ROS_ERROR_STREAM("null behavior - emergency mode");
            setEmergencyMode(true);
            ros::Rate r(1.0);
            r.sleep();
        }
    }

    delete (n);
    delete (paramNh);
    delete (reconfigServer);
    delete (mbfClient);
    delete (mbfClientExePath);
    return 0;
}

