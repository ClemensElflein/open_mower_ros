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

// Include Messages
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Polygon.h"
#include "mower_map/MapArea.h"

// Include Service Messages
#include "mower_map/AddMowingAreaSrv.h"
#include "mower_map/ClearMapSrv.h"
#include "mower_map/ClearNavPointSrv.h"
#include "mower_map/DeleteMowingAreaSrv.h"
#include "mower_map/GetDockingPointSrv.h"
#include "mower_map/GetMowingAreaSrv.h"
#include "mower_map/SetDockingPointSrv.h"
#include "mower_map/SetNavPointSrv.h"

// JSON for map storage
#include <fstream>
#include <nlohmann/json.hpp>
#include <optional>
#include <vector>
using json = nlohmann::json;

// Monitoring
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "xbot_msgs/Map.h"

// Struct definitions for JSON serialization
struct Point {
  double x;
  double y;
};

typedef std::vector<Point> Polygon;

struct MapArea {
  std::string name;
  Polygon outline;
  std::vector<Polygon> obstacles;
};

struct DockingPose {
  double x;
  double y;
  double heading;
};

struct MapData {
  std::vector<MapArea> mowing_areas;
  std::vector<MapArea> navigation_areas;
  std::optional<DockingPose> docking_pose;

  void clear() {
    mowing_areas.clear();
    navigation_areas.clear();
    docking_pose.reset();
  }
};

// JSON serialization macros
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Point, x, y)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MapArea, name, outline, obstacles)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(DockingPose, x, y, heading)

void to_json(json& j, const MapData& data) {
  j["mowing_areas"] = data.mowing_areas;
  j["navigation_areas"] = data.navigation_areas;
  if (data.docking_pose.has_value()) {
    j["docking_pose"] = data.docking_pose.value();
  }
}

void from_json(const json& j, MapData& data) {
  j.at("mowing_areas").get_to(data.mowing_areas);
  j.at("navigation_areas").get_to(data.navigation_areas);
  if (j.contains("docking_pose")) {
    data.docking_pose.emplace();
    j.at("docking_pose").get_to(data.docking_pose.value());
  }
}

// Publishes the map as occupancy grid
ros::Publisher map_pub;

// Publishes the map as markers for rviz
ros::Publisher map_server_viz_array_pub;

// Publishes map for monitoring
ros::Publisher xbot_monitoring_map_pub;

// MapData instance - the source of truth for map data
MapData map_data;

bool show_fake_obstacle = false;
geometry_msgs::Pose fake_obstacle_pose;

// The grid map. This is built from the polygons loaded from the file.
grid_map::GridMap map;

/**
 * Convert a geometry_msgs::Polygon to our internal Polygon struct
 */
Polygon geometryPolygonToInternal(const geometry_msgs::Polygon& poly) {
  Polygon result;
  for (const auto& point : poly.points) {
    result.push_back({point.x, point.y});
  }
  return result;
}

/**
 * Convert our internal Polygon struct to geometry_msgs::Polygon
 */
geometry_msgs::Polygon internalPolygonToGeometry(const Polygon& poly) {
  geometry_msgs::Polygon result;
  for (const auto& point : poly) {
    geometry_msgs::Point32 pt;
    pt.x = point.x;
    pt.y = point.y;
    result.points.push_back(pt);
  }
  return result;
}

/**
 * Convert a mower_map::MapArea to our internal MapArea struct
 */
MapArea mowerMapAreaToInternal(const mower_map::MapArea& area) {
  MapArea result;
  result.name = area.name;
  result.outline = geometryPolygonToInternal(area.area);
  for (const auto& obstacle : area.obstacles) {
    result.obstacles.push_back(geometryPolygonToInternal(obstacle));
  }
  return result;
}

/**
 * Convert our internal MapArea struct to mower_map::MapArea
 */
mower_map::MapArea internalMapAreaToMower(const MapArea& area) {
  mower_map::MapArea result;
  result.name = area.name;
  result.area = internalPolygonToGeometry(area.outline);
  for (const auto& obstacle : area.obstacles) {
    result.obstacles.push_back(internalPolygonToGeometry(obstacle));
  }
  return result;
}

/**
 * Convert a geometry_msgs::Polygon to a grid_map::Polygon.
 * This is needed in order to add recorded polys to the map.
 *
 * @param poly input poly
 * @param out result
 */
void fromMessage(geometry_msgs::Polygon& poly, grid_map::Polygon& out) {
  out.removeVertices();
  for (auto& point : poly.points) {
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

  if (map_data.docking_pose.has_value()) {
    DockingPose dp = map_data.docking_pose.value();
    xb_map.dockX = dp.x;
    xb_map.dockY = dp.y;
    xb_map.dockHeading = dp.heading;
  }

  for (const auto& area : map_data.navigation_areas) {
    xbot_msgs::MapArea xb_area;
    xb_area.name = area.name;
    xb_area.area = internalPolygonToGeometry(area.outline);
    for (const auto& obstacle : area.obstacles) {
      xb_area.obstacles.push_back(internalPolygonToGeometry(obstacle));
    }
    xb_map.navigationAreas.push_back(xb_area);
  }

  for (const auto& area : map_data.mowing_areas) {
    xbot_msgs::MapArea xb_area;
    xb_area.name = area.name;
    xb_area.area = internalPolygonToGeometry(area.outline);
    for (const auto& obstacle : area.obstacles) {
      xb_area.obstacles.push_back(internalPolygonToGeometry(obstacle));
    }
    xb_map.workingArea.push_back(xb_area);
  }

  xbot_monitoring_map_pub.publish(xb_map);
}

/**
 * Publish map visualizations for rviz.
 */
void visualizeAreas() {
  auto mapPos = map.getPosition();

  visualization_msgs::MarkerArray markerArray;

  grid_map::Polygon p;

  for (const auto& area : map_data.mowing_areas) {
    auto area_poly = internalPolygonToGeometry(area.outline);
    fromMessage(area_poly, p);
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

    for (const auto& obstacle : area.obstacles) {
      auto obstacle_poly = internalPolygonToGeometry(obstacle);
      fromMessage(obstacle_poly, p);
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
  if (map_data.docking_pose.has_value()) {
    DockingPose dp = map_data.docking_pose.value();
    geometry_msgs::Pose docking_pose;
    docking_pose.position.x = dp.x;
    docking_pose.position.y = dp.y;
    docking_pose.position.z = 0.0;

    double heading = dp.heading;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, heading);
    docking_pose.orientation = tf2::toMsg(q);

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
    marker.pose = docking_pose;
    ROS_INFO_STREAM("docking pose: " << docking_pose);
    marker.header.frame_id = "map";
    marker.ns = "mower_map_service";
    marker.id = markerArray.markers.size() + 1;
    marker.frame_locked = true;
    markerArray.markers.push_back(marker);
  }

  map_server_viz_array_pub.publish(markerArray);
}

/**
 * Set grid map values for all points in a polygon
 *
 * @param polygon The polygon containing points to set
 * @param value The value to set for each point
 * @param map The grid map to use for index calculation
 * @param data The grid map data matrix to modify
 */
void fillGridMap(const Polygon& polygon, double value, const grid_map::GridMap& map, grid_map::Matrix& data) {
  for (const auto& point : polygon) {
    grid_map::Index index;
    map.getIndex(grid_map::Position(point.x, point.y), index);
    data(index[0], index[1]) = value;
  }
}

/**
 * Uses the polygons stored in MapData to build the final occupancy grid.
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
  for (const auto& area : map_data.mowing_areas) {
    for (const auto& point : area.outline) {
      minX = std::min(minX, (float)point.x);
      maxX = std::max(maxX, (float)point.x);
      minY = std::min(minY, (float)point.y);
      maxY = std::max(maxY, (float)point.y);
    }
    for (const auto& obstacle : area.obstacles) {
      for (const auto& point : obstacle) {
        minX = std::min(minX, (float)point.x);
        maxX = std::max(maxX, (float)point.x);
        minY = std::min(minY, (float)point.y);
        maxY = std::max(maxY, (float)point.y);
      }
    }
  }
  for (const auto& area : map_data.navigation_areas) {
    for (const auto& point : area.outline) {
      minX = std::min(minX, (float)point.x);
      maxX = std::max(maxX, (float)point.x);
      minY = std::min(minY, (float)point.y);
      maxY = std::max(maxY, (float)point.y);
    }
    for (const auto& obstacle : area.obstacles) {
      for (const auto& point : obstacle) {
        minX = std::min(minX, (float)point.x);
        maxX = std::max(maxX, (float)point.x);
        minY = std::min(minY, (float)point.y);
        maxY = std::max(maxY, (float)point.y);
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
  if (map_data.mowing_areas.empty() && map_data.navigation_areas.empty()) {
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

  grid_map::Matrix& data = map["navigation_area"];
  for (const auto& area : map_data.navigation_areas) {
    fillGridMap(area.outline, 0.0, map, data);
    for (const auto& obstacle : area.obstacles) {
      fillGridMap(obstacle, 1.0, map, data);
    }
  }
  for (const auto& area : map_data.mowing_areas) {
    fillGridMap(area.outline, 0.0, map, data);
    for (const auto& obstacle : area.obstacles) {
      fillGridMap(obstacle, 1.0, map, data);
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
 * Saves the current map data to a JSON file.
 * We don't need to save the grid map, since we can easily build it again after loading.
 */
void saveMapToFile() {
  json json_data = map_data;

  // Save to JSON file
  std::ofstream file("map.json");
  if (file.is_open()) {
    file << json_data.dump(2);  // Pretty print with 2-space indentation
    file.close();
    ROS_INFO("Map saved to map.json");
  } else {
    ROS_ERROR("Failed to open map.json for writing");
  }
}

/**
 * Load the map from a JSON file and build a map.
 *
 * @param filename The file to load.
 */
void readMapFromFile(const std::string& filename) {
  std::ifstream json_file(filename);
  if (json_file.is_open()) {
    try {
      json loaded_data;
      json_file >> loaded_data;
      json_file.close();

      map_data = loaded_data;

      ROS_INFO_STREAM("Loaded " << map_data.mowing_areas.size() << " mowing areas and "
                                << map_data.navigation_areas.size() << " navigation areas from: " << filename);
    } catch (const std::exception& e) {
      ROS_ERROR_STREAM("Failed to parse " << filename << ": " << e.what());
    }
  } else {
    ROS_WARN_STREAM("Could not open map file: " << filename);
  }
}

bool addMowingArea(mower_map::AddMowingAreaSrvRequest& req, mower_map::AddMowingAreaSrvResponse& res) {
  ROS_INFO_STREAM("Got addMowingArea call");
  MapArea area = mowerMapAreaToInternal(req.area);
  if (req.isNavigationArea) {
    map_data.navigation_areas.push_back(area);
  } else {
    map_data.mowing_areas.push_back(area);
  }

  saveMapToFile();
  buildMap();
  return true;
}

bool getMowingArea(mower_map::GetMowingAreaSrvRequest& req, mower_map::GetMowingAreaSrvResponse& res) {
  ROS_INFO_STREAM("Got getMowingArea call with index: " << req.index);

  if (req.index >= map_data.mowing_areas.size()) {
    ROS_ERROR_STREAM("No mowing area with index: " << req.index);
    return false;
  }

  res.area = internalMapAreaToMower(map_data.mowing_areas[req.index]);
  return true;
}

bool deleteMowingArea(mower_map::DeleteMowingAreaSrvRequest& req, mower_map::DeleteMowingAreaSrvResponse& res) {
  ROS_INFO_STREAM("Got delete mowing area call with index: " << req.index);

  if (req.index >= map_data.mowing_areas.size()) {
    ROS_ERROR_STREAM("No mowing area with index: " << req.index);
    return false;
  }

  map_data.mowing_areas.erase(map_data.mowing_areas.begin() + req.index);

  saveMapToFile();
  buildMap();

  return true;
}

bool setDockingPoint(mower_map::SetDockingPointSrvRequest& req, mower_map::SetDockingPointSrvResponse& res) {
  ROS_INFO_STREAM("Setting Docking Point");

  // Convert quaternion to heading
  tf2::Quaternion q;
  tf2::fromMsg(req.docking_pose.orientation, q);
  tf2::Matrix3x3 m(q);
  double unused1, unused2, heading;
  m.getRPY(unused1, unused2, heading);

  map_data.docking_pose = {req.docking_pose.position.x, req.docking_pose.position.y, heading};

  saveMapToFile();
  buildMap();

  return true;
}

bool getDockingPoint(mower_map::GetDockingPointSrvRequest& req, mower_map::GetDockingPointSrvResponse& res) {
  ROS_INFO_STREAM("Getting Docking Point");

  if (!map_data.docking_pose.has_value()) {
    return false;
  }

  DockingPose docking_pose = map_data.docking_pose.value();
  res.docking_pose.position.x = docking_pose.x;
  res.docking_pose.position.y = docking_pose.y;
  res.docking_pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, docking_pose.heading);
  res.docking_pose.orientation = tf2::toMsg(q);

  return true;
}

bool setNavPoint(mower_map::SetNavPointSrvRequest& req, mower_map::SetNavPointSrvResponse& res) {
  ROS_INFO_STREAM("Setting Nav Point");

  fake_obstacle_pose = req.nav_pose;

  show_fake_obstacle = true;

  buildMap();

  return true;
}

bool clearNavPoint(mower_map::ClearNavPointSrvRequest& req, mower_map::ClearNavPointSrvResponse& res) {
  ROS_INFO_STREAM("Clearing Nav Point");

  if (show_fake_obstacle) {
    show_fake_obstacle = false;

    buildMap();
  }

  return true;
}

bool clearMap(mower_map::ClearMapSrvRequest& req, mower_map::ClearMapSrvResponse& res) {
  ROS_INFO_STREAM("Clearing Map");

  map_data.clear();

  saveMapToFile();
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "mower_map_service");
  ros::NodeHandle n;
  map_pub = n.advertise<nav_msgs::OccupancyGrid>("mower_map_service/map", 10, true);
  map_server_viz_array_pub = n.advertise<visualization_msgs::MarkerArray>("mower_map_service/map_viz", 10, true);
  xbot_monitoring_map_pub = n.advertise<xbot_msgs::Map>("xbot_monitoring/map", 10, true);

  // Load the default map file
  readMapFromFile("map.json");

  buildMap();

  ros::ServiceServer add_area_srv = n.advertiseService("mower_map_service/add_mowing_area", addMowingArea);
  ros::ServiceServer get_area_srv = n.advertiseService("mower_map_service/get_mowing_area", getMowingArea);
  ros::ServiceServer delete_area_srv = n.advertiseService("mower_map_service/delete_mowing_area", deleteMowingArea);
  ros::ServiceServer set_docking_point_srv = n.advertiseService("mower_map_service/set_docking_point", setDockingPoint);
  ros::ServiceServer get_docking_point_srv = n.advertiseService("mower_map_service/get_docking_point", getDockingPoint);
  ros::ServiceServer set_nav_point_srv = n.advertiseService("mower_map_service/set_nav_point", setNavPoint);
  ros::ServiceServer clear_nav_point_srv = n.advertiseService("mower_map_service/clear_nav_point", clearNavPoint);
  ros::ServiceServer clear_map_srv = n.advertiseService("mower_map_service/clear_map", clearMap);

  ros::spin();
  return 0;
}
