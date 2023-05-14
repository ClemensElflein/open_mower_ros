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
#include "DockingBehavior.h"

extern ros::ServiceClient dockingPointClient;
extern actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction> *mbfClient;
extern actionlib::SimpleActionClient<mbf_msgs::ExePathAction> *mbfClientExePath;
extern mower_msgs::Status getStatus();

extern void stopMoving();
extern bool setGPS(bool enabled);

DockingBehavior DockingBehavior::INSTANCE;

bool DockingBehavior::approach_docking_point() {
    ROS_INFO_STREAM("Calculating approach path");

    // Calculate a docking approaching point behind the actual docking point
    tf2::Quaternion quat;
    tf2::fromMsg(docking_pose_stamped.pose.orientation, quat);
    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);


    // Get the approach start point
    {
        geometry_msgs::PoseStamped docking_approach_point = docking_pose_stamped;
        docking_approach_point.pose.position.x -= cos(yaw) * config.docking_approach_distance;
        docking_approach_point.pose.position.y -= sin(yaw) * config.docking_approach_distance;
        mbf_msgs::MoveBaseGoal moveBaseGoal;
        moveBaseGoal.target_pose = docking_approach_point;
        moveBaseGoal.controller = "FTCPlanner";
        auto result = mbfClient->sendGoalAndWait(moveBaseGoal);
        if (result.state_ != result.SUCCEEDED) {
            return false;
        }
    }

    {
        mbf_msgs::ExePathGoal exePathGoal;

        nav_msgs::Path path;

        int dock_point_count = config.docking_approach_distance * 10.0;
        for (int i = 0; i <= dock_point_count; i++) {
            geometry_msgs::PoseStamped docking_pose_stamped_front = docking_pose_stamped;
            docking_pose_stamped_front.pose.position.x -= cos(yaw) * ((dock_point_count - i) / 10.0);
            docking_pose_stamped_front.pose.position.y -= sin(yaw) * ((dock_point_count - i) / 10.0);
            path.poses.push_back(docking_pose_stamped_front);
        }

        exePathGoal.path = path;
        exePathGoal.angle_tolerance = 1.0 * (M_PI / 180.0);
        exePathGoal.dist_tolerance = 0.1;
        exePathGoal.tolerance_from_action = true;
        exePathGoal.controller = "FTCPlanner";
        ROS_INFO_STREAM("Executing Docking Approach");

        auto approachResult = mbfClientExePath->sendGoalAndWait(exePathGoal);
        if (approachResult.state_ != approachResult.SUCCEEDED) {
            return false;
        }
    }

    return true;
}

bool DockingBehavior::dock_straight() {
    tf2::Quaternion quat;
    tf2::fromMsg(docking_pose_stamped.pose.orientation, quat);
    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);


    mbf_msgs::ExePathGoal exePathGoal;

    nav_msgs::Path path;

    int dock_point_count = config.docking_distance * 10.0;
    for (int i = 0; i < dock_point_count; i++) {
        geometry_msgs::PoseStamped docking_pose_stamped_front = docking_pose_stamped;
        docking_pose_stamped_front.pose.position.x += cos(yaw) * (i / 10.0);
        docking_pose_stamped_front.pose.position.y += sin(yaw) * (i / 10.0);
        path.poses.push_back(docking_pose_stamped_front);
    }

    exePathGoal.path = path;
    exePathGoal.angle_tolerance = 1.0 * (M_PI / 180.0);
    exePathGoal.dist_tolerance = 0.1;
    exePathGoal.tolerance_from_action = true;
    exePathGoal.controller = "DockingFTCPlanner";

    mbfClientExePath->sendGoal(exePathGoal);


    bool dockingSuccess = false;
    bool waitingForResult = true;

    ros::Rate r(10);

    // we can assume the last_state is current since we have a security timer
    while (waitingForResult) {

        r.sleep();

        const auto last_status = getStatus();
        auto mbfState = mbfClientExePath->getState();

        if(aborted) {
            ROS_INFO_STREAM("Docking aborted.");
            mbfClientExePath->cancelGoal();
            stopMoving();
            dockingSuccess = false;
            waitingForResult = false;
        }

        switch (mbfState.state_) {
            case actionlib::SimpleClientGoalState::ACTIVE:
            case actionlib::SimpleClientGoalState::PENDING:
                // currently moving. Cancel as soon as we're in the station
                if (last_status.v_charge > 5.0) {
                    ROS_INFO_STREAM("Got a voltage of " << last_status.v_charge << " V. Cancelling docking.");
                    mbfClientExePath->cancelGoal();
                    stopMoving();
                    dockingSuccess = true;
                    waitingForResult = false;
                }
                break;
            case actionlib::SimpleClientGoalState::SUCCEEDED:
                // we stopped moving because the path has ended. check, if we have docked successfully
                ROS_INFO_STREAM(
                        "Docking stopped, because we reached end pose. Voltage was " << last_status.v_charge
                                                                                     << " V.");
                if (last_status.v_charge > 5.0) {
                    mbfClientExePath->cancelGoal();
                    dockingSuccess = true;
                    stopMoving();
                }
                waitingForResult = false;
                break;
            default:
                ROS_WARN_STREAM("Some error during path execution. Docking failed. status value was: "
                                        << mbfState.state_);
                waitingForResult = false;
                stopMoving();
                break;
        }
    }

    // to be safe if the planner sent additional commands after cancel
    stopMoving();

    return dockingSuccess;
}

std::string DockingBehavior::state_name() {
    return "DOCKING";
}

Behavior *DockingBehavior::execute() {

    // Check if already docked (e.g. carried to base during emergency) and skip
    if(getStatus().v_charge > 5.0) {
        ROS_INFO_STREAM("Already inside docking station, going directly to idle.");
        stopMoving();
        return &IdleBehavior::INSTANCE;
    }

    while(!isGPSGood){
        ROS_WARN_STREAM("Waiting for good GPS");
        ros::Duration(1.0).sleep();
    }

    bool approachSuccess = approach_docking_point();

    if (!approachSuccess) {
        ROS_ERROR("Error during docking approach.");

        retryCount++;
        if(retryCount <= config.docking_retry_count) {
            ROS_ERROR("Retrying docking approach");
            return &DockingBehavior::INSTANCE;
        }

        ROS_ERROR("Giving up on docking");
        return &IdleBehavior::INSTANCE;
    }

    // Reset retryCount
    reset();

    // Disable GPS
    inApproachMode = false;
    setGPS(false);

    bool docked = dock_straight();

    if (!docked) {
        ROS_ERROR("Error during docking.");

        retryCount++;
        if(retryCount <= config.docking_retry_count && !aborted) {
            ROS_ERROR_STREAM("Retrying docking. Try " << retryCount << " / " << config.docking_retry_count);
            return &UndockingBehavior::RETRY_INSTANCE;
        }

        ROS_ERROR("Giving up on docking");
        return &IdleBehavior::INSTANCE;
    }

    return &IdleBehavior::INSTANCE;
}

void DockingBehavior::enter() {
    paused = aborted = false;
    // start with target approach and then dock later
    inApproachMode = true;

    // Get the docking pose in map
    mower_map::GetDockingPointSrv get_docking_point_srv;
    dockingPointClient.call(get_docking_point_srv);
    docking_pose_stamped.pose = get_docking_point_srv.response.docking_pose;
    docking_pose_stamped.header.frame_id = "map";
    docking_pose_stamped.header.stamp = ros::Time::now();
}

void DockingBehavior::exit() {

}

void DockingBehavior::reset() {
    retryCount = 0;
}

bool DockingBehavior::needs_gps() {
    // we only need GPS if we're in approach mode
    return inApproachMode;
}

bool DockingBehavior::mower_enabled() {
    // No mower during docking
    return false;
}

void DockingBehavior::command_home() {

}

void DockingBehavior::command_start() {

}

void DockingBehavior::command_s1() {

}

void DockingBehavior::command_s2() {

}

bool DockingBehavior::redirect_joystick() {
    return false;
}


uint8_t DockingBehavior::get_sub_state() {
    return 1;

}
uint8_t DockingBehavior::get_state() {
    return mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_AUTONOMOUS;
}

void DockingBehavior::handle_action(std::string action) {

}

