// Created by Clemens Elflein on 2/18/22, 5:37 PM.
// Copyright (c) 2022 Clemens Elflein and OpenMower contributors. All rights reserved.
//
// This file is part of OpenMower.
//
// OpenMower is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
// License as published by the Free Software Foundation, version 3 of the License.
//
// OpenMower is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with OpenMower. If not, see
// <https://www.gnu.org/licenses/>.
//
#include "grid_map_cv/GridMapCvConverter.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "grid_map_ros/PolygonRosConverter.hpp"
#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"

// Rosbag for reading/writing the map to a file
#include <rosbag/bag.h>
#include <rosbag/view.h>

// Include Messages
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/PoseStamped.h"
#include "mower_map/MapArea.h"
#include "mower_map/MapAreas.h"

// Include Service Messages
#include "mower_map/AddMowingAreaSrv.h"
#include "mower_map/AppendMapSrv.h"
#include "mower_map/ClearMapSrv.h"
#include "mower_map/ClearNavPointSrv.h"
#include "mower_map/ConvertToNavigationAreaSrv.h"
#include "mower_map/DeleteMowingAreaSrv.h"
#include "mower_map/GetDockingPointSrv.h"
#include "mower_map/GetMowingAreaSrv.h"
#include "mower_map/SetDockingPointSrv.h"
#include "mower_map/SetNavPointSrv.h"

// Monitoring
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "xbot_msgs/Map.h"

// Publishes the map as occupancy grid
ros::Publisher map_pub, map_areas_pub;

// Publishes the map as markers for rviz
ros::Publisher map_server_viz_array_pub;

// Publishes map for monitoring
ros::Publisher xbot_monitoring_map_pub;

// We store navigation_areas (i.e. robot is allowed to move here) and
// mowing_areas (i.e. grass needs to be cut here)
std::vector<mower_map::MapArea> navigation_areas;
std::vector<mower_map::MapArea> mowing_areas;

// The recorded docking pose. Note that this is the pose from which the docking attempt is started
// I.e. the robot will drive to this pose and then drive forward
geometry_msgs::Pose docking_point;
bool has_docking_point = false;
bool show_fake_obstacle = false;
geometry_msgs::Pose fake_obstacle_pose;

// The grid map. This is built from the polygons loaded from the file.
grid_map::GridMap map;

/**
 * Convert a geometry_msgs::Polygon to a grid_map::Polygon.
 * This is needed in order to add recorded polys to the map.
 *
 * @param poly input poly
 * @param out result
 */
void fromMessage(geometry_msgs::Polygon &poly, grid_map::Polygon &out) {
  out.removeVertices();
  for (auto &point : poly.points) {
    grid_map::Position pos;
    pos.x() = point.x;
    pos.y() = point.y;
    out.addVertex(pos);
  }
}

/**
 * Publish map to xbot_monitoring
 */
void publishMapMonitoring() {
  xbot_msgs::Map xb_map;
  xb_map.mapWidth = map.getSize().x() * map.getResolution();
  xb_map.mapHeight = map.getSize().y() * map.getResolution();
  auto mapPos = map.getPosition();
  xb_map.mapCenterX = mapPos.x();
  xb_map.mapCenterY = mapPos.y();

  xb_map.dockX = docking_point.position.x;
  xb_map.dockY = docking_point.position.y;
  tf2::Quaternion q;
  tf2::fromMsg(docking_point.orientation, q);

  tf2::Matrix3x3 m(q);
  double unused1, unused2, yaw;

  m.getRPY(unused1, unused2, yaw);
  xb_map.dockHeading = yaw;

  for (const auto &area : navigation_areas) {
    xbot_msgs::MapArea xb_area;
    xb_area.name = area.name;
    xb_area.area = area.area;
    xb_area.obstacles = area.obstacles;
    xb_map.navigationAreas.push_back(xb_area);
  }
  for (const auto &area : mowing_areas) {
    xbot_msgs::MapArea xb_area;
    xb_area.name = area.name;
    xb_area.area = area.area;
    xb_area.obstacles = area.obstacles;
    xb_map.workingArea.push_back(xb_area);
  }

  xbot_monitoring_map_pub.publish(xb_map);
}

/**
 * Publish map visualizations for rviz.
 */
void visualizeAreas() {
  mower_map::MapAreas mapAreas;

  mapAreas.mapWidth = map.getSize().x() * map.getResolution();
  mapAreas.mapHeight = map.getSize().y() * map.getResolution();
  auto mapPos = map.getPosition();
  mapAreas.mapCenterX = mapPos.x();
  mapAreas.mapCenterY = mapPos.y();

  visualization_msgs::MarkerArray markerArray;

  grid_map::Polygon p;

  for (const auto &navigationArea : navigation_areas) {
    mapAreas.navigationAreas.push_back(navigationArea);
  }

  for (auto mowingArea : mowing_areas) {
    // Push it to mapAreas
    mapAreas.mowingAreas.push_back(mowingArea);
    {
      // Create a marker
      fromMessage(mowingArea.area, p);
      std_msgs::ColorRGBA color;
      color.g = 1.0;
      color.a = 1.0;
      visualization_msgs::Marker marker;
      grid_map::PolygonRosConverter::toLineMarker(p, color, 0.05, 0, marker);

      marker.header.frame_id = "map";
      marker.ns = "mower_map_service";
      marker.id = markerArray.markers.size();
      marker.frame_locked = true;
      marker.pose.orientation.w = 1.0;

      markerArray.markers.push_back(marker);
    }
    for (auto obstacle : mowingArea.obstacles) {
      fromMessage(obstacle, p);
      std_msgs::ColorRGBA color;
      color.r = 1.0;
      color.a = 1.0;
      visualization_msgs::Marker marker;
      grid_map::PolygonRosConverter::toLineMarker(p, color, 0.05, 0, marker);

      marker.header.frame_id = "map";
      marker.ns = "mower_map_service";
      marker.id = markerArray.markers.size();
      marker.frame_locked = true;

      marker.pose.orientation.w = 1.0;
      markerArray.markers.push_back(marker);
    }
  }

  // Visualize Docking Point
  {
    std_msgs::ColorRGBA color;
    color.b = 1.0;
    color.a = 1.0;
    visualization_msgs::Marker marker;

    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.2;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color = color;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.pose = docking_point;
    ROS_INFO_STREAM("docking pose: " << docking_point);
    marker.header.frame_id = "map";
    marker.ns = "mower_map_service";
    marker.id = markerArray.markers.size() + 1;
    marker.frame_locked = true;
    markerArray.markers.push_back(marker);
  }

  map_server_viz_array_pub.publish(markerArray);
  map_areas_pub.publish(mapAreas);
}

/**
 * Uses the polygons stored in navigation_areas and mowing_areas to build the final occupancy grid.
 *
 * First, the map is marked as completely occupied. Then navigation_areas and mowing_areas are marked as free.
 *
 * Then, all obstacles are marked as occupied.
 *
 * Finally, a blur is applied to the map so that it is expensive, but not completely forbidden to drive near boundaries.
 */
void buildMap() {
  // First, calculate the size of the map by finding the min and max values for x and y.
  float minX = FLT_MAX;
  float maxX = -FLT_MAX;
  float minY = FLT_MAX;
  float maxY = -FLT_MAX;

  // loop through all areas and calculate a size where everything fits
  for (const auto &area : mowing_areas) {
    for (auto pt : area.area.points) {
      minX = std::min(minX, pt.x);
      maxX = std::max(maxX, pt.x);
      minY = std::min(minY, pt.y);
      maxY = std::max(maxY, pt.y);
    }
    for (const auto &obstacle : area.obstacles) {
      for (const auto &pt : obstacle.points) {
        minX = std::min(minX, pt.x);
        maxX = std::max(maxX, pt.x);
        minY = std::min(minY, pt.y);
        maxY = std::max(maxY, pt.y);
      }
    }
  }
  for (const auto &area : navigation_areas) {
    for (auto pt : area.area.points) {
      minX = std::min(minX, pt.x);
      maxX = std::max(maxX, pt.x);
      minY = std::min(minY, pt.y);
      maxY = std::max(maxY, pt.y);
    }
    for (const auto &obstacle : area.obstacles) {
      for (const auto &pt : obstacle.points) {
        minX = std::min(minX, pt.x);
        maxX = std::max(maxX, pt.x);
        minY = std::min(minY, pt.y);
        maxY = std::max(maxY, pt.y);
      }
    }
  }

  // Enlarge the map by 1m in all directions.
  // This guarantees that even after blurring, the map has an occupied border.
  maxX += 1.0;
  minX -= 1.0;
  maxY += 1.0;
  minY -= 1.0;

  // Check, if the map was empty. If so, we'd create a huge map. Therefore we build an empty 10x10m map instead.
  if (mowing_areas.empty() && navigation_areas.empty()) {
    maxX = 5.0;
    minX = -5.0;
    maxY = 5.0;
    minY = -5.0;
  }

  map = grid_map::GridMap({"navigation_area"});
  map.setFrameId("map");
  grid_map::Position origin;
  origin.x() = (maxX + minX) / 2.0;
  origin.y() = (maxY + minY) / 2.0;

  ROS_INFO_STREAM("Map Position: x=" << origin.x() << ", y=" << origin.y());
  ROS_INFO_STREAM("Map Size: x=" << (maxX - minX) << ", y=" << (maxY - minY));

  map.setGeometry(grid_map::Length(maxX - minX, maxY - minY), 0.05, origin);
  map.setTimestamp(ros::Time::now().toNSec());

  map.clearAll();
  map["navigation_area"].setConstant(1.0);

  grid_map::Matrix &data = map["navigation_area"];
  for (auto mowingArea : navigation_areas) {
    grid_map::Polygon poly;
    fromMessage(mowingArea.area, poly);

    for (grid_map::PolygonIterator iterator(map, poly); !iterator.isPastEnd(); ++iterator) {
      const grid_map::Index index(*iterator);
      data(index[0], index[1]) = 0.0;
    }
    for (auto obstacle : mowingArea.obstacles) {
      fromMessage(obstacle, poly);
      for (grid_map::PolygonIterator iterator(map, poly); !iterator.isPastEnd(); ++iterator) {
        const grid_map::Index index(*iterator);
        data(index[0], index[1]) = 1.0;
      }
    }
  }
  for (auto mowingArea : mowing_areas) {
    grid_map::Polygon poly;
    fromMessage(mowingArea.area, poly);

    for (grid_map::PolygonIterator iterator(map, poly); !iterator.isPastEnd(); ++iterator) {
      const grid_map::Index index(*iterator);
      data(index[0], index[1]) = 0.0;
    }
    for (auto obstacle : mowingArea.obstacles) {
      fromMessage(obstacle, poly);
      for (grid_map::PolygonIterator iterator(map, poly); !iterator.isPastEnd(); ++iterator) {
        const grid_map::Index index(*iterator);
        data(index[0], index[1]) = 1.0;
      }
    }
  }

  if (show_fake_obstacle) {
    grid_map::Polygon poly;
    tf2::Quaternion q;
    tf2::fromMsg(fake_obstacle_pose.orientation, q);

    tf2::Matrix3x3 m(q);
    double unused1, unused2, yaw;

    m.getRPY(unused1, unused2, yaw);

    Eigen::Vector2d front(cos(yaw), sin(yaw));
    Eigen::Vector2d left(-sin(yaw), cos(yaw));
    Eigen::Vector2d obstacle_pos(fake_obstacle_pose.position.x, fake_obstacle_pose.position.y);

    {
      grid_map::Position pos = obstacle_pos + 0.1 * left + 0.25 * front;
      poly.addVertex(pos);
    }
    {
      grid_map::Position pos = obstacle_pos + 0.2 * left - 0.1 * front;
      poly.addVertex(pos);
    }
    {
      grid_map::Position pos = obstacle_pos + 0.6 * left - 0.1 * front;
      poly.addVertex(pos);
    }
    {
      grid_map::Position pos = obstacle_pos + 0.6 * left + 0.7 * front;
      poly.addVertex(pos);
    }

    {
      grid_map::Position pos = obstacle_pos - 0.6 * left + 0.7 * front;
      poly.addVertex(pos);
    }
    {
      grid_map::Position pos = obstacle_pos - 0.6 * left - 0.1 * front;
      poly.addVertex(pos);
    }
    {
      grid_map::Position pos = obstacle_pos - 0.2 * left - 0.1 * front;
      poly.addVertex(pos);
    }
    {
      grid_map::Position pos = obstacle_pos - 0.1 * left + 0.25 * front;
      poly.addVertex(pos);
    }
    for (grid_map::PolygonIterator iterator(map, poly); !iterator.isPastEnd(); ++iterator) {
      const grid_map::Index index(*iterator);
      data(index[0], index[1]) = 1.0;
    }
  }

  cv::Mat cv_map;
  grid_map::GridMapCvConverter::toImage<unsigned char, 1>(map, "navigation_area", CV_8UC1, cv_map);

  cv::blur(cv_map, cv_map, cv::Size(5, 5));

  grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(cv_map, "navigation_area", map);

  nav_msgs::OccupancyGrid msg;
  grid_map::GridMapRosConverter::toOccupancyGrid(map, "navigation_area", 0.0, 1.0, msg);
  map_pub.publish(msg);

  publishMapMonitoring();
  visualizeAreas();
}

/**
 * Saves the current polygons to a bag file.
 * We don't need to save the map, since we can easily build it again after loading.
 */
void saveMapToFile() {
  rosbag::Bag bag;
  bag.open("map.bag", rosbag::bagmode::Write);

  for (auto &area : mowing_areas) {
    bag.write("mowing_areas", ros::Time::now(), area);
  }
  for (auto &area : navigation_areas) {
    bag.write("navigation_areas", ros::Time::now(), area);
  }

  if (has_docking_point) {
    bag.write("docking_point", ros::Time::now(), docking_point);
  }

  bag.close();
}

/**
 * Load the polygons from the bag file and build a map.
 *
 * @param filename The file to load.
 * @param append True to append the loaded map to the current one.
 */
void readMapFromFile(const std::string &filename, bool append = false) {
  if (!append) {
    mowing_areas.clear();
    navigation_areas.clear();
  }
  rosbag::Bag bag;
  try {
    bag.open(filename);
  } catch (rosbag::BagIOException &e) {
    ROS_WARN("Error opening stored mowing areas.");
    return;
  }

  {
    rosbag::View view(bag, rosbag::TopicQuery("mowing_areas"));

    for (rosbag::MessageInstance const m : view) {
      auto area = m.instantiate<mower_map::MapArea>();
      mowing_areas.push_back(*area);
    }
  }
  {
    rosbag::View view(bag, rosbag::TopicQuery("navigation_areas"));

    for (rosbag::MessageInstance const m : view) {
      auto area = m.instantiate<mower_map::MapArea>();
      navigation_areas.push_back(*area);
    }
  }

  {
    has_docking_point = false;
    rosbag::View view(bag, rosbag::TopicQuery("docking_point"));
    for (rosbag::MessageInstance const m : view) {
      auto pt = m.instantiate<geometry_msgs::Pose>();
      docking_point = *pt;
      has_docking_point = true;
      break;
    }
    if (!has_docking_point) {
      geometry_msgs::Pose empty;
      empty.orientation.w = 1.0;
      docking_point = empty;
    }
  }

  ROS_INFO_STREAM("Loaded " << mowing_areas.size() << " mowing areas and " << navigation_areas.size()
                            << " navigation areas from file.");
}

bool addMowingArea(mower_map::AddMowingAreaSrvRequest &req, mower_map::AddMowingAreaSrvResponse &res) {
  ROS_INFO_STREAM("Got addMowingArea call");

  if (req.isNavigationArea) {
    navigation_areas.push_back(req.area);
  } else {
    mowing_areas.push_back(req.area);
  }

  saveMapToFile();
  buildMap();
  return true;
}

bool getMowingArea(mower_map::GetMowingAreaSrvRequest &req, mower_map::GetMowingAreaSrvResponse &res) {
  ROS_INFO_STREAM("Got getMowingArea call with index: " << req.index);

  if (req.index >= mowing_areas.size()) {
    ROS_ERROR_STREAM("No mowing area with index: " << req.index);
    return false;
  }

  res.area = mowing_areas.at(req.index);

  return true;
}

bool deleteMowingArea(mower_map::DeleteMowingAreaSrvRequest &req, mower_map::DeleteMowingAreaSrvResponse &res) {
  ROS_INFO_STREAM("Got delete mowing area call with index: " << req.index);

  if (req.index >= mowing_areas.size()) {
    ROS_ERROR_STREAM("No mowing area with index: " << req.index);
    return false;
  }

  mowing_areas.erase(mowing_areas.begin() + req.index);

  saveMapToFile();
  buildMap();

  return true;
}

bool convertToNavigationArea(mower_map::ConvertToNavigationAreaSrvRequest &req,
                             mower_map::ConvertToNavigationAreaSrvResponse &res) {
  ROS_INFO_STREAM("Got convert to nav area call with index: " << req.index);

  if (req.index >= mowing_areas.size()) {
    ROS_ERROR_STREAM("No mowing area with index: " << req.index);
    return false;
  }

  navigation_areas.push_back(mowing_areas[req.index]);

  mowing_areas.erase(mowing_areas.begin() + req.index);

  saveMapToFile();
  buildMap();

  return true;
}

bool appendMapFromFile(mower_map::AppendMapSrvRequest &req, mower_map::AppendMapSrvResponse &res) {
  ROS_INFO_STREAM("Appending maps from: " << req.bagfile);

  readMapFromFile(req.bagfile, true);

  saveMapToFile();
  buildMap();

  return true;
}

bool setDockingPoint(mower_map::SetDockingPointSrvRequest &req, mower_map::SetDockingPointSrvResponse &res) {
  ROS_INFO_STREAM("Setting Docking Point");

  docking_point = req.docking_pose;
  has_docking_point = true;

  saveMapToFile();
  buildMap();

  return true;
}

bool getDockingPoint(mower_map::GetDockingPointSrvRequest &req, mower_map::GetDockingPointSrvResponse &res) {
  ROS_INFO_STREAM("Getting Docking Point");

  res.docking_pose = docking_point;

  return has_docking_point;
}

bool setNavPoint(mower_map::SetNavPointSrvRequest &req, mower_map::SetNavPointSrvResponse &res) {
  ROS_INFO_STREAM("Setting Nav Point");

  fake_obstacle_pose = req.nav_pose;

  show_fake_obstacle = true;

  buildMap();

  return true;
}

bool clearNavPoint(mower_map::ClearNavPointSrvRequest &req, mower_map::ClearNavPointSrvResponse &res) {
  ROS_INFO_STREAM("Clearing Nav Point");

  if (show_fake_obstacle) {
    show_fake_obstacle = false;

    buildMap();
  }

  return true;
}

bool clearMap(mower_map::ClearMapSrvRequest &req, mower_map::ClearMapSrvResponse &res) {
  ROS_INFO_STREAM("Clearing Map");

  mowing_areas.clear();
  navigation_areas.clear();
  has_docking_point = false;

  saveMapToFile();
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "mower_map_service");
  has_docking_point = false;
  ros::NodeHandle n;
  map_pub = n.advertise<nav_msgs::OccupancyGrid>("mower_map_service/map", 10, true);
  map_areas_pub = n.advertise<mower_map::MapAreas>("mower_map_service/map_areas", 10, true);
  map_server_viz_array_pub = n.advertise<visualization_msgs::MarkerArray>("mower_map_service/map_viz", 10, true);
  xbot_monitoring_map_pub = n.advertise<xbot_msgs::Map>("xbot_monitoring/map", 10, true);

  // Load the default map file
  readMapFromFile("map.bag");

  buildMap();

  ros::ServiceServer add_area_srv = n.advertiseService("mower_map_service/add_mowing_area", addMowingArea);
  ros::ServiceServer get_area_srv = n.advertiseService("mower_map_service/get_mowing_area", getMowingArea);
  ros::ServiceServer delete_area_srv = n.advertiseService("mower_map_service/delete_mowing_area", deleteMowingArea);
  ros::ServiceServer append_maps_srv = n.advertiseService("mower_map_service/append_maps", appendMapFromFile);
  ros::ServiceServer convert_maps_srv =
      n.advertiseService("mower_map_service/convert_to_navigation_area", convertToNavigationArea);
  ros::ServiceServer set_docking_point_srv = n.advertiseService("mower_map_service/set_docking_point", setDockingPoint);
  ros::ServiceServer get_docking_point_srv = n.advertiseService("mower_map_service/get_docking_point", getDockingPoint);
  ros::ServiceServer set_nav_point_srv = n.advertiseService("mower_map_service/set_nav_point", setNavPoint);
  ros::ServiceServer clear_nav_point_srv = n.advertiseService("mower_map_service/clear_nav_point", clearNavPoint);
  ros::ServiceServer clear_map_srv = n.advertiseService("mower_map_service/clear_map", clearMap);

  ros::spin();
  return 0;
}
