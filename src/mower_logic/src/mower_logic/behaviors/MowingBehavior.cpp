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
#include "mower_map/GetMowingAreaSrv.h"
#include "mower_map/SetNavPointSrv.h"
#include "mower_map/ClearNavPointSrv.h"
#include "MowingBehavior.h"
#include "mower_logic/CheckPoint.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <nav_msgs/Path.h>
#include <cryptopp/cryptlib.h>
#include <cryptopp/sha.h>
#include <cryptopp/hex.h>

extern ros::ServiceClient mapClient;
extern ros::ServiceClient pathClient;
extern ros::ServiceClient pathProgressClient;
extern ros::ServiceClient setNavPointClient;
extern ros::ServiceClient clearNavPointClient;

extern actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction> *mbfClient;
extern actionlib::SimpleActionClient<mbf_msgs::ExePathAction> *mbfClientExePath;
extern mower_logic::MowerLogicConfig getConfig();
extern void setConfig(mower_logic::MowerLogicConfig);

extern void registerActions(std::string prefix, const std::vector<xbot_msgs::ActionInfo> &actions);

MowingBehavior MowingBehavior::INSTANCE;

std::string MowingBehavior::state_name() {
    if(paused) {
        return "PAUSED";
    }
    return "MOWING";
}

Behavior *MowingBehavior::execute() {
    shared_state->active_semiautomatic_task = true;

    while (ros::ok() && !aborted) {
        if (currentMowingPaths.empty() && !create_mowing_plan(currentMowingArea)) {
            ROS_INFO_STREAM("MowingBehavior: Could not create mowing plan, docking");
            // Start again from first area next time.
            reset();
            // We cannot create a plan, so we're probably done. Go to docking station
            return &DockingBehavior::INSTANCE;
        }

        // We have a plan, execute it
        ROS_INFO_STREAM("MowingBehavior: Executing mowing plan");
        bool finished = execute_mowing_plan();
        if (finished) {
            // skip to next area if current
            ROS_INFO_STREAM("MowingBehavior: Executing mowing plan - finished");
            currentMowingArea++;
            currentMowingPaths.clear();
            currentMowingPath = 0;
            currentMowingPathIndex = 0;
        }
    }

    if (!ros::ok()) {
        // something went wrong
        return nullptr;
    }
    // we got aborted, go to docking station
    return &DockingBehavior::INSTANCE;
}

void MowingBehavior::enter() {
    skip_area = false;
    skip_path = false;
    paused = aborted = false;

    for(auto& a : actions) {
        a.enabled = true;
    }
    registerActions("mower_logic:mowing", actions);
}

void MowingBehavior::exit() {
    for(auto& a : actions) {
        a.enabled = false;
    }
    registerActions("mower_logic:mowing", actions);
}

void MowingBehavior::reset() {
    currentMowingPaths.clear();
    currentMowingArea = 0;
    currentMowingPath = 0;
    currentMowingPathIndex = 0;
    // increase cumulative mowing angle offset increment
    currentMowingAngleIncrementSum = std::fmod(currentMowingAngleIncrementSum + getConfig().mow_angle_increment, 360);
    checkpoint();

    if (config.automatic_mode == eAutoMode::SEMIAUTO) {
        ROS_INFO_STREAM("MowingBehavior: Finished semiautomatic task");
        shared_state->active_semiautomatic_task = false;
    }
}

bool MowingBehavior::needs_gps() {
    return true;
}

bool MowingBehavior::mower_enabled() {
    return mowerEnabled;
}

void MowingBehavior::update_actions() {
    for(auto& a : actions) {
        a.enabled = true;
    }

    // pause / resume switch. other actions are always available
    actions[0].enabled = !paused &&  !requested_pause_flag;
    actions[1].enabled = paused && !requested_continue_flag;

    registerActions("mower_logic:mowing", actions);
}

bool MowingBehavior::create_mowing_plan(int area_index) {
    ROS_INFO_STREAM("MowingBehavior: Creating mowing plan for area: " << area_index);
    // Delete old plan and progress.
    currentMowingPaths.clear();

    // get the mowing area
    mower_map::GetMowingAreaSrv mapSrv;
    mapSrv.request.index = area_index;
    if (!mapClient.call(mapSrv)) {
        ROS_ERROR_STREAM("MowingBehavior: Error loading mowing area");
        return false;
    }

    // Area orientation is the same as the first point
    double angle = 0;
    auto points = mapSrv.response.area.area.points;
    if (points.size() >= 2) {
        tf2::Vector3 first(points[0].x, points[0].y, 0);
        for(auto point : points) {
            tf2::Vector3 second(point.x, point.y, 0);
            auto diff = second - first;
            if(diff.length() > 2.0) {
                // we have found a point that has a distance of > 1 m, calculate the angle
                angle = atan2(diff.y(), diff.x());
                ROS_INFO_STREAM("MowingBehavior: Detected mow angle: " << angle);
                break;
            }
        }
    }

    // add mowing angle offset increment and return into the <-180, 180> range
    double mow_angle_offset = std::fmod(getConfig().mow_angle_offset + currentMowingAngleIncrementSum + 180, 360);
    if (mow_angle_offset < 0) mow_angle_offset += 360;
    mow_angle_offset -= 180;
    ROS_INFO_STREAM("MowingBehavior: mowing angle offset (deg): " << mow_angle_offset);
    if (config.mow_angle_offset_is_absolute) {
        angle = mow_angle_offset * (M_PI / 180.0);
        ROS_INFO_STREAM("MowingBehavior: Custom mowing angle: " << angle);
    } else {
        angle = angle + mow_angle_offset * (M_PI / 180.0);
        ROS_INFO_STREAM("MowingBehavior: Auto-detected mowing angle + mowing angle offset: " << angle);
    }

    // calculate coverage
    slic3r_coverage_planner::PlanPath pathSrv;
    pathSrv.request.angle = angle;
    pathSrv.request.outline_count = config.outline_count;
    pathSrv.request.outline_overlap_count = config.outline_overlap_count;
    pathSrv.request.outline = mapSrv.response.area.area;
    pathSrv.request.holes = mapSrv.response.area.obstacles;
    pathSrv.request.fill_type = slic3r_coverage_planner::PlanPathRequest::FILL_LINEAR;
    pathSrv.request.outer_offset = config.outline_offset;
    pathSrv.request.distance = config.tool_width;
    if (!pathClient.call(pathSrv)) {
        ROS_ERROR_STREAM("MowingBehavior: Error during coverage planning");
        return false;
    }

    currentMowingPaths = pathSrv.response.paths;

    // Calculate mowing plan digest from the poses
    // TODO: move to slic3r_coverage_planner
    CryptoPP::SHA256 hash;
    byte digest[CryptoPP::SHA256::DIGESTSIZE];
    for(const auto &path:currentMowingPaths) {
        for(const auto &pose_stamped:path.path.poses) {
            hash.Update(reinterpret_cast<const byte*>(&pose_stamped.pose), sizeof(geometry_msgs::Pose));
        }
    }
    hash.Final((byte*)&digest[0]);
    CryptoPP::HexEncoder encoder;
    std::string mowingPlanDigest="";
    encoder.Attach( new CryptoPP::StringSink(mowingPlanDigest) );
    encoder.Put( digest, sizeof(digest) );
    encoder.MessageEnd();

    // Proceed to checkpoint?
    if(mowingPlanDigest == currentMowingPlanDigest) {
        ROS_INFO_STREAM("MowingBehavior: Advancing to checkpoint, path: " << currentMowingPath << " index: " << currentMowingPathIndex);
    } else {
        ROS_INFO_STREAM(
            "MowingBehavior: Ignoring checkpoint for plan ("
            << currentMowingPlanDigest <<
            ") current mowing plan is ("
            << mowingPlanDigest
            << ")"
        );
        // Plan has changed so must restart the area
        currentMowingPlanDigest = mowingPlanDigest;
        currentMowingPath = 0;
        currentMowingPathIndex = 0;
    }

    return true;
}

int getCurrentMowPathIndex()
{
    ftc_local_planner::PlannerGetProgress progressSrv;
    int currentIndex = -1;
    if(pathProgressClient.call(progressSrv)) {
        currentIndex = progressSrv.response.index;
    } else {
        ROS_ERROR("MowingBehavior: getMowIndex() - Error getting progress from FTC planner");
    }
    return(currentIndex);
}

void printNavState(int state)
{
    switch (state)
    {
        case actionlib::SimpleClientGoalState::PENDING: ROS_INFO(">>> State: Pending <<<"); break;
        case actionlib::SimpleClientGoalState::ACTIVE: ROS_INFO(">>> State: Active <<<"); break;
        case actionlib::SimpleClientGoalState::RECALLED: ROS_INFO(">>> State: Recalled <<<"); break;
        case actionlib::SimpleClientGoalState::REJECTED: ROS_INFO(">>> State: Rejected <<<"); break;
        case actionlib::SimpleClientGoalState::PREEMPTED: ROS_INFO(">>> State: Preempted <<<"); break;
        case actionlib::SimpleClientGoalState::ABORTED: ROS_INFO(">>> State: Aborted <<<"); break;
        case actionlib::SimpleClientGoalState::SUCCEEDED: ROS_INFO(">>> State: Succeeded <<<"); break;
        case actionlib::SimpleClientGoalState::LOST: ROS_INFO(">>> State: Lost <<<"); break;
        default: ROS_INFO(">>> State: Unknown Hu ? <<<"); break;
    }
}

bool MowingBehavior::execute_mowing_plan() {

    int first_point_attempt_counter = 0;
    int first_point_trim_counter = 0;
    ros::Time paused_time(0.0);

    // loop through all mowingPaths to execute the plan fully.
    while (currentMowingPath < currentMowingPaths.size() && ros::ok() && !aborted) {
        ////////////////////////////////////////////////
        // PAUSE HANDLING
        ////////////////////////////////////////////////
        if (requested_pause_flag)
        {  // pause was requested
            this->setPause();  // set paused=true
            update_actions();
            mowerEnabled = false;
            while (!requested_continue_flag) // while not asked to continue, we wait
            {
                ROS_INFO_STREAM("MowingBehavior: PAUSED (waiting for CONTINUE)");
                ros::Rate r(1.0);
                r.sleep();
            }
            // we will drop into paused, thus will also wait for /odom to be valid again
        }
        if (paused)
        {   
            paused_time = ros::Time::now();
            mowerEnabled = false;
            while (!this->hasGoodGPS()) // while no good GPS we wait
            {
                ROS_INFO_STREAM("MowingBehavior: PAUSED (" << (ros::Time::now()-paused_time).toSec() << "s) (waiting for /odom)");
                ros::Rate r(1.0);
                r.sleep();
            }
            ROS_INFO_STREAM("MowingBehavior: CONTINUING");
            this->setContinue();
            update_actions();
            mowerEnabled = true;
        }
    

        auto &path = currentMowingPaths[currentMowingPath];
        ROS_INFO_STREAM("MowingBehavior: Path segment length: " << path.path.poses.size() << " poses.");

        // Check if path is empty. If so, directly skip it
        if(path.path.poses.size() - currentMowingPathIndex <= 0) {
            ROS_INFO_STREAM("MowingBehavior: Skipping empty path.");
            currentMowingPath++;
            currentMowingPathIndex = 0;
            continue;
        }

        /////////////////////////////////////////////////////////////////////////////////////////////////////////
        // DRIVE TO THE FIRST POINT OF THE MOW PATH
        //
        // * we have n attempts, if we fail we go to pause() mode because most likely it was GPS problems that 
        //   prevented us from reaching the inital pose
        // * after n attempts, we fail the mow area and skip to the next one
        /////////////////////////////////////////////////////////////////////////////////////////////////////////
        {
            ROS_INFO_STREAM("MowingBehavior: (FIRST POINT)  Moving to path segment starting point");
            if(path.is_outline && getConfig().add_fake_obstacle) {
                mower_map::SetNavPointSrv set_nav_point_srv;
                set_nav_point_srv.request.nav_pose = path.path.poses[currentMowingPathIndex].pose;
                setNavPointClient.call(set_nav_point_srv);
                sleep(1);
            }

            mbf_msgs::MoveBaseGoal moveBaseGoal;
            moveBaseGoal.target_pose = path.path.poses[currentMowingPathIndex];
            moveBaseGoal.controller = "FTCPlanner";
            mbfClient->sendGoal(moveBaseGoal);
            sleep(1);
            actionlib::SimpleClientGoalState current_status(actionlib::SimpleClientGoalState::PENDING);
            ros::Rate r(10);

            // wait for path execution to finish
            while (ros::ok()) {
                current_status = mbfClient->getState();
                if (current_status.state_ == actionlib::SimpleClientGoalState::ACTIVE ||
                    current_status.state_ == actionlib::SimpleClientGoalState::PENDING) {
                    // path is being executed, everything seems fine.
                    // check if we should pause or abort mowing
                    if(skip_area) {
                        ROS_INFO_STREAM("MowingBehavior: (FIRST POINT) SKIP AREA was requested.");
                        // remove all paths in current area and return true
                        mowerEnabled = false;
                        mbfClientExePath->cancelAllGoals();
                        currentMowingPaths.clear();
                        skip_area = false;
                        return true;
                    }
                    if(skip_path) {
                        skip_path= false;
                        currentMowingPath++;
                        currentMowingPathIndex = 0;
                        return false;
                    }
                    if (aborted) {
                        ROS_INFO_STREAM("MowingBehavior: (FIRST POINT) ABORT was requested - stopping path execution.");
                        mbfClientExePath->cancelAllGoals();
                        mowerEnabled = false;
                        return false;
                    }
                    if (requested_pause_flag) {
                        ROS_INFO_STREAM("MowingBehavior: (FIRST POINT) PAUSE was requested - stopping path execution.");
                        mbfClientExePath->cancelAllGoals();
                        mowerEnabled = false;
                        return false;
                    }
                } else {
                    ROS_INFO_STREAM("MowingBehavior: (FIRST POINT)  Got status " << current_status.state_ << " from MBF/FTCPlanner -> Stopping path execution.");
                    // we're done, break out of the loop
                    break;
                }
                r.sleep();
            }

            first_point_attempt_counter++;
            if (current_status.state_ != actionlib::SimpleClientGoalState::SUCCEEDED) {
                // we cannot reach the start point
                ROS_ERROR_STREAM("MowingBehavior: (FIRST POINT) - Could not reach goal (first point). Planner Status was: " << current_status.state_);
                // we have 3 attempts to get to the start pose of the mowing area
                if (first_point_attempt_counter < config.max_first_point_attempts)
                {
                    ROS_WARN_STREAM("MowingBehavior: (FIRST POINT) - Attempt " << first_point_attempt_counter << " / " << config.max_first_point_attempts << " Making a little pause ...");
                    this->setPause();
                    update_actions();
                }
                else
                {
                    // We failed to reach the first point in the mow path by simply repeating the drive to process
                    // So now we will trim the path by removing the first pose
                    if (first_point_trim_counter < config.max_first_point_trim_attempts)
                    {
                        // We try now to remove the first point so the 2nd, 3rd etc point becomes our target
                        // mow path points are offset by 10cm
                        ROS_WARN_STREAM("MowingBehavior: (FIRST POINT) - Attempt " << first_point_trim_counter << " / " << config.max_first_point_trim_attempts << " Trimming first point off the beginning of the mow path.");
                        currentMowingPathIndex++;
                        first_point_trim_counter++;
                        first_point_attempt_counter = 0; // give it another <config.max_first_point_attempts> attempts
                        this->setPause();
                        update_actions();
                    }
                    else
                    {
                        // Unable to reach the start of the mow path (we tried multiple attempts for the same point, and we skipped points which also didnt work, time to give up) 
                        ROS_ERROR_STREAM("MowingBehavior: (FIRST POINT) Max retries reached, we are unable to reach any of the first points - aborting at index: "
                            << currentMowingPathIndex << " path: " << currentMowingPath << " area: " << currentMowingArea );
                        this->abort();
                    }
                }
                continue;
            }

            mower_map::ClearNavPointSrv clear_nav_point_srv;
            clearNavPointClient.call(clear_nav_point_srv);

            // we have reached the start pose of the mow area, reset error handling values
            first_point_attempt_counter = 0;
            first_point_trim_counter = 0;
        }
        
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Execute the path segment and either drop it if we finished it successfully or trim it if we were aborted
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        {
            // enable mower (only when we reach the start not on the way to mowing already)
            mowerEnabled = true;

            mbf_msgs::ExePathGoal exePathGoal;
            nav_msgs::Path exePath;
            exePath.header = path.path.header;
            exePath.poses = std::vector<geometry_msgs::PoseStamped>(path.path.poses.begin() + currentMowingPathIndex, path.path.poses.end());
            int exePathStartIndex = currentMowingPathIndex;
            exePathGoal.path = exePath;
            exePathGoal.angle_tolerance = 5.0 * (M_PI / 180.0);
            exePathGoal.dist_tolerance = 0.2;
            exePathGoal.tolerance_from_action = true;
            exePathGoal.controller = "FTCPlanner";

            ROS_INFO_STREAM("MowingBehavior: (MOW) First point reached - Executing mow path with " << path.path.poses.size() << " poses, from index " << exePathStartIndex);
            mbfClientExePath->sendGoal(exePathGoal);
            sleep(1);
            actionlib::SimpleClientGoalState current_status(actionlib::SimpleClientGoalState::PENDING);
            ros::Rate r(10);

            // wait for path execution to finish
            while (ros::ok()) {
                current_status = mbfClientExePath->getState();
                if (current_status.state_ == actionlib::SimpleClientGoalState::ACTIVE ||
                    current_status.state_ == actionlib::SimpleClientGoalState::PENDING) {
                    // path is being executed, everything seems fine.
                    // check if we should pause or abort mowing
                    if(skip_area) {
                        ROS_INFO_STREAM("MowingBehavior: (MOW) SKIP AREA was requested.");
                        // remove all paths in current area and return true
                        mowerEnabled = false;
                        currentMowingPaths.clear();
                        skip_area = false;
                        return true;
                    }
                    if(skip_path) {
                        skip_path= false;
                        currentMowingPath++;
                        currentMowingPathIndex = 0;
                        return false;
                    }
                    if (aborted) {
                        ROS_INFO_STREAM("MowingBehavior: (MOW) ABORT was requested - stopping path execution.");
                        mbfClientExePath->cancelAllGoals();
                        mowerEnabled = false;
                        break; // Trim path
                    }
                    if (requested_pause_flag) {
                        ROS_INFO_STREAM("MowingBehavior: (MOW) PAUSE was requested - stopping path execution.");
                        mbfClientExePath->cancelAllGoals();
                        mowerEnabled = false;
                        break; // Trim path
                    }
                    if(current_status.state_ == actionlib::SimpleClientGoalState::ACTIVE) {
                        // show progress
                        int currentIndex = getCurrentMowPathIndex();
                        if (currentIndex != -1) {
                            currentMowingPathIndex = exePathStartIndex + currentIndex;
                        }
                        ROS_INFO_STREAM_THROTTLE(5, "MowingBehavior: (MOW) Progress: " << currentMowingPathIndex << "/" << path.path.poses.size());
                        if ( ros::Time::now() - last_checkpoint > ros::Duration(30.0) ) checkpoint();
                    }
                } else {
                    ROS_INFO_STREAM("MowingBehavior: (MOW)  Got status " << current_status.state_ << " from MBF/FTCPlanner -> Stopping path execution.");
                    // we're done, break out of the loop
                    break;
                }
                r.sleep();
            } 

            // Only skip/trim if goal execution began
            if (current_status.state_ != actionlib::SimpleClientGoalState::PENDING &&
                current_status.state_ != actionlib::SimpleClientGoalState::RECALLED)
            {
                ROS_INFO_STREAM(">> MowingBehavior: (MOW) PlannerGetProgress currentMowingPathIndex = " << currentMowingPathIndex << " of " << path.path.poses.size());
                printNavState(current_status.state_);
                // if we have fully processed the segment or we have encountered an error, drop the path segment
                /* TODO: we can not trust the SUCCEEDED state because the planner sometimes says suceeded with
                    the currentIndex far from the size of the poses ! (BUG in planner ?)
                    instead we trust only the currentIndex vs. poses.size() */
                if (currentMowingPathIndex >= path.path.poses.size() || (path.path.poses.size() - currentMowingPathIndex) < 5) // fully mowed the path ?
                {
                    ROS_INFO_STREAM("MowingBehavior: (MOW) Mow path finished, skipping to next mow path.");
                    currentMowingPath++;
                    currentMowingPathIndex = 0;
                    // continue with next segment
                }
                else
                {
                    // we didnt drive all points in the mow path, so we go into pause mode
                    // TODO: we should figure out the likely reason for our failure to complete the path
                    // if GPS -> PAUSE
                    // if something else -> Recovery Behaviour ?

                    // currentMowingPathIndex might be 0 if we never consumed one of the points, we advance at least 1 point
                    if( currentMowingPathIndex == 0) currentMowingPathIndex++;
                    ROS_INFO_STREAM("MowingBehavior: (MOW) PAUSED due to MBF Error at " << currentMowingPathIndex);
                    this->setPause();
                    update_actions();
                }
            }
        }
    }

    mowerEnabled = false;

    // true, if we have executed all paths
    return currentMowingPath >= currentMowingPaths.size();
}

void MowingBehavior::command_home() {
    if(shared_state->active_semiautomatic_task) {
        // We are in semiautomatic task, mark it as manually paused.
        ROS_INFO_STREAM("Manually pausing semiautomatic task");
        shared_state->semiautomatic_task_paused = true;
    }
    if (paused)
    {
        // Request continue to wait for odom
        this->requestContinue();
        // Then instantly abort i.e. go to dock.
    }
    this->abort();
}

void MowingBehavior::command_start() {
    ROS_INFO_STREAM("MowingBehavior: MANUAL CONTINUE");
    if(shared_state->active_semiautomatic_task && shared_state->semiautomatic_task_paused) {
        // We are in semiautomatic task and paused, user wants to resume, so store that immediately.
        // This way, once we are docked the mower will continue as soon as all other conditions are g2g
        ROS_INFO_STREAM("Resuming semiautomatic task");
        shared_state->semiautomatic_task_paused = true;
    }
    this->requestContinue();
}

void MowingBehavior::command_s1() {
    ROS_INFO_STREAM("MowingBehavior: MANUAL PAUSED");
    this->requestPause();
}

void MowingBehavior::command_s2() {
    skip_area = true;
}

bool MowingBehavior::redirect_joystick() {
    return false;
}


uint8_t MowingBehavior::get_sub_state() {
    return 0;

}
uint8_t MowingBehavior::get_state() {
    return mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_AUTONOMOUS;
}

MowingBehavior::MowingBehavior() {
    last_checkpoint = ros::Time(0.0);
    xbot_msgs::ActionInfo pause_action;
    pause_action.action_id = "pause";
    pause_action.enabled = false;
    pause_action.action_name = "Pause Mowing";

    xbot_msgs::ActionInfo continue_action;
    continue_action.action_id = "continue";
    continue_action.enabled = false;
    continue_action.action_name = "Continue Mowing";

    xbot_msgs::ActionInfo abort_mowing_action;
    abort_mowing_action.action_id = "abort_mowing";
    abort_mowing_action.enabled = false;
    abort_mowing_action.action_name = "Stop Mowing";

    xbot_msgs::ActionInfo skip_area_action;
    skip_area_action.action_id = "skip_area";
    skip_area_action.enabled = false;
    skip_area_action.action_name = "Skip Area";

    xbot_msgs::ActionInfo skip_path_action;
    skip_path_action.action_id = "skip_path";
    skip_path_action.enabled = false;
    skip_path_action.action_name = "Skip Path";

    actions.clear();
    actions.push_back(pause_action);
    actions.push_back(continue_action);
    actions.push_back(abort_mowing_action);
    actions.push_back(skip_area_action);

    restore_checkpoint();
}

void MowingBehavior::handle_action(std::string action) {
    if(action == "mower_logic:mowing/pause") {
        ROS_INFO_STREAM("got pause command");
        this->requestPause();
    }else if(action == "mower_logic:mowing/continue") {
        ROS_INFO_STREAM("got continue command");
        this->requestContinue();
    } else if(action == "mower_logic:mowing/abort_mowing") {
        ROS_INFO_STREAM("got abort mowing command");
        command_home();
    } else if(action == "mower_logic:mowing/skip_area") {
        ROS_INFO_STREAM("got skip_area command");
        skip_area = true;
    } else if(action == "mower_logic:mowing/skip_path") {
        ROS_INFO_STREAM("got skip_path command");
        skip_path = true;
    }
    update_actions();
}

void MowingBehavior::checkpoint() {
    rosbag::Bag bag;
    mower_logic::CheckPoint cp;
    cp.currentMowingPath = currentMowingPath;
    cp.currentMowingArea = currentMowingArea;
    cp.currentMowingPathIndex = currentMowingPathIndex;
    cp.currentMowingPlanDigest = currentMowingPlanDigest;
    cp.currentMowingAngleIncrementSum = currentMowingAngleIncrementSum;
    bag.open("checkpoint.bag", rosbag::bagmode::Write);
    bag.write("checkpoint", ros::Time::now(), cp);
    bag.close();
    last_checkpoint = ros::Time::now();
}

bool MowingBehavior::restore_checkpoint() {
    rosbag::Bag bag;
    bool found = false;
    try {
        bag.open("checkpoint.bag");
    } catch (rosbag::BagIOException &e) {
        // Checkpoint does not exist or is corrupt, start at the very beginning
        currentMowingArea = 0;
        currentMowingPath = 0;
        currentMowingPathIndex = 0;
        currentMowingAngleIncrementSum = 0;
        return false;
    }
    {
        rosbag::View view(bag, rosbag::TopicQuery("checkpoint"));
        for (rosbag::MessageInstance const m: view) {
            auto cp = m.instantiate<mower_logic::CheckPoint>();
            if(cp) {
                ROS_INFO_STREAM(
                    "Restoring checkpoint for plan ("
                    << cp->currentMowingPlanDigest
                    << ")"
                    << " area: " << cp->currentMowingArea
                    << " path: " << cp->currentMowingPath
                    << " index: " << cp->currentMowingPathIndex
                    << " angle increment sum: " << cp->currentMowingAngleIncrementSum
                );
                currentMowingPath = cp->currentMowingPath;
                currentMowingArea = cp->currentMowingArea;
                currentMowingPathIndex = cp->currentMowingPathIndex;
                currentMowingPlanDigest = cp->currentMowingPlanDigest;
                currentMowingAngleIncrementSum = cp->currentMowingAngleIncrementSum;
                found = true;
                break;
            }
        }
        bag.close();
    }
    return found;
}
