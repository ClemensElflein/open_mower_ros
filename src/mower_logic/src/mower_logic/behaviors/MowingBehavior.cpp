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
    return "MOWING";
}

Behavior *MowingBehavior::execute() {
    auto config = getConfig();
    if (config.clear_path_on_start) {
        currentMowingPaths.clear();
        config.clear_path_on_start = false;
        setConfig(config);
    }

    shared_state->active_semiautomatic_task = true;

    while (ros::ok() && !aborted) {
        if (currentMowingPaths.empty() && !create_mowing_plan(getConfig().current_area)) {
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
            auto config = getConfig();
            config.current_area++;
            setConfig(config);
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
    auto config = getConfig();
    config.current_area = 0;

    if (config.automatic_mode == eAutoMode::SEMIAUTO) {
        ROS_INFO_STREAM("MowingBehavior: Finished semiautomatic task");
        shared_state->active_semiautomatic_task = false;
    }

    // increment mowing angle offset and return into the <-180, 180> range
    config.mow_angle_offset = std::fmod(config.mow_angle_offset + config.mow_angle_increment + 180, 360);
    if (config.mow_angle_offset < 0) config.mow_angle_offset += 360;
    config.mow_angle_offset -= 180;

    setConfig(config);
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

    // handling mowing angle offset
    ROS_INFO_STREAM("MowingBehavior: mowing angle offset: " << (config.mow_angle_offset * (M_PI / 180.0)));
    if (config.mow_angle_offset_is_absolute) {
        angle = config.mow_angle_offset * (M_PI / 180.0);
        ROS_INFO_STREAM("MowingBehavior: Custom mowing angle: " << angle);
    } else {
        angle = angle + config.mow_angle_offset * (M_PI / 180.0);
        ROS_INFO_STREAM("MowingBehavior: Auto-detected mowing angle + mowing angle offset: " << angle);
    }

    // calculate coverage
    slic3r_coverage_planner::PlanPath pathSrv;
    pathSrv.request.angle = angle;
    pathSrv.request.outline_count = config.outline_count;
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
    while (!currentMowingPaths.empty() && ros::ok() && !aborted) {
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


        auto &path = currentMowingPaths.front();
        ROS_INFO_STREAM("MowingBehavior: Path segment length: " << path.path.poses.size() << " poses.");

        // Check if path is empty. If so, directly skip it
        if(path.path.poses.size() == 0) {
            ROS_INFO_STREAM("MowingBehavior: Skipping empty path.");
            currentMowingPaths.erase(currentMowingPaths.begin());
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
                set_nav_point_srv.request.nav_pose = path.path.poses.front().pose;
                setNavPointClient.call(set_nav_point_srv);
                sleep(1);
            }

            mbf_msgs::MoveBaseGoal moveBaseGoal;
            moveBaseGoal.target_pose = path.path.poses.front();
            moveBaseGoal.controller = "FTCPlanner";
            auto result = mbfClient->sendGoalAndWait(moveBaseGoal);
            first_point_attempt_counter++;
            if (result.state_ != result.SUCCEEDED) {
                // we cannot reach the start point
                ROS_ERROR_STREAM("MowingBehavior: (FIRST POINT) - Could not reach goal (first point). Planner Status was: " << result.state_);
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
                        auto &poses = path.path.poses;
                        ROS_WARN_STREAM("MowingBehavior: (FIRST POINT) - Attempt " << first_point_trim_counter << " / " << config.max_first_point_trim_attempts << " Trimming first point off the beginning of the mow path.");
                        poses.erase(poses.begin(), poses.begin() + 1);
                        first_point_trim_counter++;
                        first_point_attempt_counter = 0; // give it another <config.max_first_point_attempts> attempts
                        this->setPause();
                        update_actions();
                    }
                    else
                    {
                        // Unable to reach the start of the mow path (we tried multiple attempts for the same point, and we skipped points which also didnt work, time to give up) 
                        ROS_ERROR_STREAM("MowingBehavior: (FIRST POINT) Max retries reached, we are unable to reach any of the first points - aborting this mow area ...");
                        currentMowingPaths.erase(currentMowingPaths.begin());
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
            exePathGoal.path = path.path;
            exePathGoal.angle_tolerance = 5.0 * (M_PI / 180.0);
            exePathGoal.dist_tolerance = 0.2;
            exePathGoal.tolerance_from_action = true;
            exePathGoal.controller = "FTCPlanner";

            ROS_INFO_STREAM("MowingBehavior: (MOW) First point reached - Executing mow path with " << path.path.poses.size() << " poses");            
            mbfClientExePath->sendGoal(exePathGoal);
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
                    // show progress
                    ROS_INFO_STREAM_THROTTLE(5, "MowingBehavior: (MOW) Progress: " << getCurrentMowPathIndex() << "/" << path.path.poses.size());                    
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
                int currentIndex = getCurrentMowPathIndex();
                ROS_INFO_STREAM(">> MowingBehavior: (MOW) PlannerGetProgress currentIndex = " << currentIndex << " of " << path.path.poses.size());
                printNavState(current_status.state_);
                // if we have fully processed the segment or we have encountered an error, drop the path segment
                /* TODO: we can not trust the SUCCEEDED state because the planner sometimes says suceeded with
                    the currentIndex far from the size of the poses ! (BUG in planner ?)
                    instead we trust only the currentIndex vs. poses.size() */
                if (currentIndex >= path.path.poses.size() || (path.path.poses.size() - currentIndex) < 5) // fully mowed the path ?
                {
                    ROS_INFO_STREAM("MowingBehavior: (MOW) Mow path finished, skipping to next mow path.");
                    currentMowingPaths.erase(currentMowingPaths.begin());
                    // continue with next segment
                }
                else
                {
                    // we didnt drive all points in the mow path, so we go into pause mode
                    // TODO: we should figure out the likely reason for our failure to complete the path
                    // if GPS -> PAUSE
                    // if something else -> Recovery Behaviour ?
                    auto &poses = path.path.poses;
                    ROS_INFO_STREAM("MowingBehavior (ErrorCatch): Poses before trim:" << poses.size());
                    if (currentIndex == 0) // currentIndex might be 0 if we never consumed one of the points, we trim at least 1 point
                    {
                        currentIndex = 1;
                    }
                    ROS_INFO_STREAM("MowingBehavior (ErrorCatch): Trimming " << currentIndex << " points.");
                    poses.erase(poses.begin(), poses.begin() + currentIndex);
                    ROS_INFO_STREAM("MowingBehavior (ErrorCatch): Poses after trim:" << poses.size());
                    ROS_INFO_STREAM("MowingBehavior: (MOW) PAUSED due to MBF Error");
                    this->setPause();
                    update_actions();
                }
            }
        }
    }

    mowerEnabled = false;

    // true, if we have executed all paths
    return currentMowingPaths.empty();
}

void MowingBehavior::command_home() {
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

    actions.clear();
    actions.push_back(pause_action);
    actions.push_back(continue_action);
    actions.push_back(abort_mowing_action);
    actions.push_back(skip_area_action);
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
        if (paused)
        {
            // Request continue to wait for odom
            this->requestContinue();
            // Then instantly abort i.e. go to dock.
        }
        this->abort();
    } else if(action == "mower_logic:mowing/skip_area") {
        ROS_INFO_STREAM("got skip_area command");
        skip_area = true;
    }
    update_actions();
}
