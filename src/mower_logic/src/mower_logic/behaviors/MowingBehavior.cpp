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
#include <dynamic_reconfigure/server.h>
#include "MowingBehavior.h"


extern ros::ServiceClient mapClient;
extern ros::ServiceClient pathClient;
extern ros::ServiceClient pathProgressClient;
extern ros::ServiceClient setNavPointClient;
extern ros::ServiceClient clearNavPointClient;

extern actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction> *mbfClient;
extern actionlib::SimpleActionClient<mbf_msgs::ExePathAction> *mbfClientExePath;
extern mower_logic::MowerLogicConfig last_config;
extern dynamic_reconfigure::Server<mower_logic::MowerLogicConfig> *reconfigServer;

MowingBehavior MowingBehavior::INSTANCE;

std::string MowingBehavior::state_name() {
    return "MOWING";
}

Behavior *MowingBehavior::execute() {

    while (ros::ok() && !aborted) {
        if (currentMowingPaths.empty() && !create_mowing_plan(last_config.current_area)) {
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
            last_config.current_area++;
            reconfigServer->updateConfig(last_config);
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
}

void MowingBehavior::exit() {

}

void MowingBehavior::reset() {
    currentMowingPaths.clear();
    last_config.current_area = 0;
    reconfigServer->updateConfig(last_config);
}

bool MowingBehavior::needs_gps() {
    return true;
}

bool MowingBehavior::mower_enabled() {
    return mowerEnabled;
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

bool MowingBehavior::execute_mowing_plan() {

    // loop through all mowingPaths to execute the plan fully.
    while (!currentMowingPaths.empty() && ros::ok() && !aborted) {
        ////////////////////////////////////////////////
        // PAUSE HANDLING
        ////////////////////////////////////////////////
        if (paused)
        {   
            if (request_pause)
            {  // pause was requested
                while (!request_continue) // while paused we wait
                {
                    ROS_INFO_STREAM("MowingBehavior: PAUSED (waiting for CONTINUE)");
                    ros::Rate r(1.0);
                    r.sleep();
                }
                while (!this->isOdomValid()) // while we have no /odom we wait
                {
                    ROS_INFO_STREAM("MowingBehavior: CONTINUED (but waiting for /odom)");
                    ros::Rate r(1.0);
                    r.sleep();
                }
            }
            else
            {   // pause was not requested, but forced 
                while (!this->isOdomValid()) // while no /odom we wait
                {
                    ROS_INFO_STREAM("MowingBehavior: PAUSED (waiting for /odom)");
                    ros::Rate r(1.0);
                    r.sleep();
                }
            }
            ROS_INFO_STREAM("MowingBehavior: CONTINUING");
            this->setContinue();
        }


        ROS_INFO_STREAM("MowingBehavior: Moving to path segment starting point");
        // enable mower (MOWGLI update: see below we only start when we reach the inital position of the mow path)
        // mowerEnabled = true;

        auto &path = currentMowingPaths.front();
        ROS_INFO_STREAM("MowingBehavior: Path segment length: " << path.path.poses.size() << " poses.");

        // Drive to first point of the path segment
        {
            if(path.is_outline && last_config.add_fake_obstacle) {
                mower_map::SetNavPointSrv set_nav_point_srv;
                set_nav_point_srv.request.nav_pose = path.path.poses.front().pose;
                setNavPointClient.call(set_nav_point_srv);
                sleep(1);
            }

            mbf_msgs::MoveBaseGoal moveBaseGoal;
            moveBaseGoal.target_pose = path.path.poses.front();
            moveBaseGoal.controller = "FTCPlanner";
            auto result = mbfClient->sendGoalAndWait(moveBaseGoal);
            if (result.state_ != result.SUCCEEDED) {
                // We cannot reach the start point, drop the current path segment
                ROS_ERROR_STREAM("MowingBehavior: Could not reach goal (first point), quitting. status was: " << result.state_);
                currentMowingPaths.erase(currentMowingPaths.begin());
                continue;
            }

            mower_map::ClearNavPointSrv clear_nav_point_srv;
            clearNavPointClient.call(clear_nav_point_srv);
        }
        
        // Execute the path segment and either drop it if we finished it successfully or trim it if we were aborted
        {
             // enable mower (MOWGLI update - only when we reach the start not on the way to mowing already)
            mowerEnabled = true;

            mbf_msgs::ExePathGoal exePathGoal;
            exePathGoal.path = path.path;
            exePathGoal.angle_tolerance = 5.0 * (M_PI / 180.0);
            exePathGoal.dist_tolerance = 0.2;
            exePathGoal.tolerance_from_action = true;
            exePathGoal.controller = "FTCPlanner";

            ROS_INFO_STREAM("MowingBehavior: First point reached, executing path segment with " << path.path.poses.size() << " poses");            
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
                    if (aborted || skip_area) {
                        ROS_INFO_STREAM("MowingBehavior: ABORT was requested - stopping path execution.");
                        mbfClientExePath->cancelAllGoals();
                        break;
                    }
                    if (request_pause) {
                        ROS_INFO_STREAM("MowingBehavior: PAUSE was requested - stopping path execution.");
                        mbfClientExePath->cancelAllGoals();
                        break;
                    }
                } else {
                    ROS_INFO_STREAM("MowingBehavior: Got status " << current_status.state_ << " from MBF/FTCPlanner -> Stopping path execution.");
                    // we're done, break out of the loop
                    break;
                }
                r.sleep();
            } 

            if(skip_area) {
                ROS_INFO_STREAM("MowingBehavior: skip_area = true");
                // remove all paths in current area and return true
                mowerEnabled = false;
                currentMowingPaths.clear();
                skip_area = false;
                return true;
            }

            // We are done processing a path segment. Either trim it or remove it if it was processed fully
         //   ftc_local_planner::PlannerGetProgress progressSrv;
         //   int currentIndex = -1;
         //   if(pathProgressClient.call(progressSrv)) {
         //      currentIndex = progressSrv.response.index;
         //   } else {
         //       ROS_ERROR("MowingBehavior: Error getting progress from FTC planner");
         //   }

            int currentIndex = getCurrentMowPathIndex();
            ROS_INFO_STREAM(">> MowingBehavior: PlannerGetProgress currentIndex = " << currentIndex);
            ROS_INFO_STREAM(">> MowingBehavior: path.path.poses.size() = " << path.path.poses.size());
            ROS_INFO_STREAM(">> MowingBehavior: aborted = " << aborted);

            // if we have fully processed the segment or we have encountered an error, drop the path segment
            if (currentIndex < 0 || currentIndex >= path.path.poses.size() || !aborted) {
                // TODO check if it was an error and maybe react to it
                if (current_status.state_ != actionlib::SimpleClientGoalState::SUCCEEDED) {
                    ROS_INFO_STREAM("MowingBehavior: ErrorCatch triggered");
                    // remove currentMowingPathIndex points from the path, since we have already mowed those
                    auto &poses = path.path.poses;
                    ROS_INFO_STREAM("MowingBehavior (ErrorCatch): Poses before trim:" << poses.size());
                    ROS_INFO_STREAM("MowingBehavior (ErrorCatch): Trimming " << currentIndex << " points.");
                    poses.erase(poses.begin(), poses.begin() + currentIndex);
                    ROS_INFO_STREAM("MowingBehavior (ErrorCatch): Poses after trim:" << poses.size());
                    ROS_INFO_STREAM("MowingBehavior: PAUSED due to MBF Error");
                   
                    this->setPause();
                }
                else
                {
                    ROS_INFO_STREAM("MowingBehavior: Path segment finished, skipping to next.");
                    currentMowingPaths.erase(currentMowingPaths.begin());
                    // continue with next segment
                }
                continue;
            }

            ROS_INFO_STREAM("MowingBehavior: Path segment was not fully finished, trimming it.");

            // remove currentMowingPathIndex points from the path, since we have already mowed those
            auto &poses = path.path.poses;
            ROS_INFO_STREAM("MowingBehavior: Poses before trim:" << poses.size());
            ROS_INFO_STREAM("MowingBehavior: Trimming " << currentIndex << " points.");
            poses.erase(poses.begin(), poses.begin() + currentIndex);
            ROS_INFO_STREAM("MowingBehavior: Poses after trim:" << poses.size());
        }
    }

    mowerEnabled = false;

    // true, if we have executed all paths
    return currentMowingPaths.empty();
}

void MowingBehavior::command_home() {
    if (paused)
    {
        this->requestContinue();
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
