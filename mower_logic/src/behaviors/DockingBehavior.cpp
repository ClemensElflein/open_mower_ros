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
extern mower_msgs::Status last_status;

extern void stop();

extern bool setGPS(bool enabled);

DockingBehavior DockingBehavior::INSTANCE;

bool DockingBehavior::approach_docking_point() {
    mbf_msgs::MoveBaseGoal moveBaseGoal;
    moveBaseGoal.target_pose = docking_pose_stamped;

    auto result = mbfClient->sendGoalAndWait(moveBaseGoal);
    if (result.state_ != result.SUCCEEDED) {
        return false;
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

        auto mbfState = mbfClientExePath->getState();

        switch (mbfState.state_) {
            case actionlib::SimpleClientGoalState::ACTIVE:
            case actionlib::SimpleClientGoalState::PENDING:
                // currently moving. Cancel as soon as we're in the station
                if (last_status.v_charge > 5.0) {
                    ROS_INFO_STREAM("Got a voltage of " << last_status.v_charge << " V. Cancelling docking.");
                    mbfClientExePath->cancelGoal();
                    stop();
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
                    stop();
                }
                waitingForResult = false;
                break;
            default:
                ROS_WARN_STREAM("Some error during path execution. Docking failed. status value was: "
                                        << mbfState.state_);
                waitingForResult = false;
                stop();
                break;
        }
    }

    // to be safe if the planner sent additional commands after cancel
    stop();

    return dockingSuccess;
}

std::string DockingBehavior::state_name() {
    return "DOCKING";
}

Behavior *DockingBehavior::execute() {

    bool approachSuccess = approach_docking_point();

    if (!approachSuccess) {
        ROS_ERROR("Error during docking approach. Quitting to emergency mode.");
        return nullptr;
    }

    // Disable GPS
    inApproachMode = false;
    setGPS(false);

    bool docked = dock_straight();

    if (!docked) {
        ROS_ERROR_STREAM("Error during docking.");
        return nullptr;
    }

    return &IdleBehavior::INSTANCE;
}

void DockingBehavior::enter() {
    reset();

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
    // start with target approach and then dock later
    inApproachMode = true;
}

bool DockingBehavior::needs_gps() {
    // we only need GPS if we're in approach mode
    return inApproachMode;
}

bool DockingBehavior::mower_enabled() {
    // No mower during docking
    return false;
}