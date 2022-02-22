//
// Created by Clemens Elflein on 22.02.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//
#include "ros/ros.h"
#include "slic3r_coverage_planner/PlanPath.h"
#include "mower_map/GetMowingAreaSrv.h"

ros::ServiceClient pathClient, mapClient;

int main(int argc, char **argv) {
    ros::init(argc, argv, "planner_test");

    ros::NodeHandle n;
    ros::NodeHandle paramNh("~");


    ros::Publisher path_pub;

    path_pub = n.advertise<nav_msgs::Path>("mower_logic/mowing_path", 100, true);

    pathClient = n.serviceClient<slic3r_coverage_planner::PlanPath>(
            "slic3r_coverage_planner/plan_path");
    mapClient = n.serviceClient<mower_map::GetMowingAreaSrv>(
            "mower_map_service/get_mowing_area");


    ROS_INFO("Waiting for map server");
    if (!mapClient.waitForExistence(ros::Duration(60.0, 0.0))) {
        ROS_ERROR("Map server service not found.");
        return 2;
    }


    mower_map::GetMowingAreaSrv mapSrv;
    mapSrv.request.index = 0;

    if (!mapClient.call(mapSrv)) {
        ROS_ERROR_STREAM("Error loading mowing area");
        return 1;
    }

    slic3r_coverage_planner::PlanPath pathSrv;
    pathSrv.request.angle = 0;
    pathSrv.request.outline_count = 1;
    pathSrv.request.outline = mapSrv.response.area.area;
    pathSrv.request.holes = mapSrv.response.area.obstacles;
    pathSrv.request.fill_type = slic3r_coverage_planner::PlanPathRequest::FILL_LINEAR;
    pathSrv.request.distance = 1.0;

    if (!pathClient.call(pathSrv)) {
        ROS_ERROR_STREAM("Error getting path area");
        return 1;
    }
    return 0;
}

