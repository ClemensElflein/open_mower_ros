//
// Created by Clemens Elflein on 22.11.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//
#include <filesystem>

#include "ros/ros.h"
#include "xbot_msgs/SensorInfo.h"
#include "xbot_msgs/SensorDataDouble.h"
#include "xbot_msgs/AbsolutePose.h"
#include "grid_map_core/GridMap.hpp"
#include <grid_map_ros/GridMapRosConverter.hpp>
#include "xbot_msgs/Map.h"
#include <boost/algorithm/algorithm.hpp>
#include <boost/algorithm/string/erase.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

ros::NodeHandle *n;

xbot_msgs::AbsolutePose last_pose{};
std::unordered_map<std::string, std::pair<grid_map::GridMap, ros::Publisher>> pubsub_map{};
xbot_msgs::Map lastMap{};
bool has_map = false;
void onMap(const xbot_msgs::Map &mapInfo) {
    if(!has_map || lastMap.mapCenterY != mapInfo.mapCenterY || lastMap.mapCenterX != mapInfo.mapCenterX || lastMap.mapWidth != mapInfo.mapWidth || lastMap.mapHeight != mapInfo.mapHeight) {
        ROS_INFO("Updated heatmap geometry.");
        for(auto & [key,value] : pubsub_map) {
                value.first.setGeometry(grid_map::Length(mapInfo.mapWidth, mapInfo.mapHeight), 0.2,
                                        grid_map::Position(mapInfo.mapCenterX, mapInfo.mapCenterY));
        }
        has_map = true;
        lastMap = mapInfo;
    }
}

void onSensorData(const std::string sensor_id, const xbot_msgs::SensorDataDouble::ConstPtr &msg) {
    auto & [map, publisher] = pubsub_map[sensor_id];
    try {
        map.atPosition("intensity",
                       grid_map::Position(last_pose.pose.pose.position.x, last_pose.pose.pose.position.y)) = msg->data;
        map.atPosition("elevation",
                       grid_map::Position(last_pose.pose.pose.position.x, last_pose.pose.pose.position.y)) = 0;
    } catch (std::exception &e) {
        // error inserting into map, skip it
        ROS_WARN_STREAM("Could not add point to heatmap: " << e.what());
        return;
    }

    sensor_msgs::PointCloud2 mapmsg;
    grid_map::GridMapRosConverter::toPointCloud(map, map.getLayers(), "elevation", mapmsg);
    publisher.publish(mapmsg);
}

void onRobotPos(const xbot_msgs::AbsolutePose ::ConstPtr &msg) {
    last_pose = *msg;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "heatmap_generator");


    n = new ros::NodeHandle();
    ros::NodeHandle paramNh("~");

    std::vector<std::string> sensor_ids;
    std::string sensor_ids_string;
    if(!paramNh.getParam("sensor_ids", sensor_ids_string)) {
        ROS_ERROR("Please specify sensor_ids for heatmap generation as comma separated list.");
        return 1;
    }

    boost::erase_all(sensor_ids_string, " ");
    boost::split ( sensor_ids, sensor_ids_string, boost::is_any_of(","));

    // Subscribe to robot position
    ros::Subscriber stateSubscriber = n->subscribe("xbot_positioning/xb_pose", 10, onRobotPos);

    // Subscribe to map info
    ros::Subscriber mapSubscriber = n->subscribe("xbot_monitoring/map", 1, onMap);

    std::vector<ros::Subscriber> sensorSubscribers{};
    for(const auto & sensor_id : sensor_ids)
    {
        ROS_INFO_STREAM("Generating heatmap for sensor id: " << sensor_id);
        // Add layer to map
        grid_map::GridMap map{};
        map.add("intensity");
        map.add("elevation");
        map.setFrameId("map");
        // Create a publisher for the heatmap
        ros::Publisher p = n->advertise<sensor_msgs::PointCloud2>("heatmap/"+sensor_id, 10, true);
        // Subscribe to sensor data
        ros::Subscriber s = n->subscribe<xbot_msgs::SensorDataDouble>("xbot_monitoring/sensors/" + sensor_id + "/data", 10,
                                                        [sensor_id](const xbot_msgs::SensorDataDouble::ConstPtr &msg) {
                                                            onSensorData(sensor_id, msg);
                                                        });
        // store in map, so that they don't get deconstructed
        pubsub_map[sensor_id] = std::make_pair(std::move(map), std::move(p));
        sensorSubscribers.push_back(std::move(s));
    }

    ros::spin();
    return 0;
}
