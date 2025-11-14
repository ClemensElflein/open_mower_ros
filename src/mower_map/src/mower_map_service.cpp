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
#include "std_msgs/String.h"
#include "visualization_msgs/MarkerArray.h"

// Rosbag for reading/writing the map to a file
#include <rosbag/bag.h>
#include <rosbag/view.h>

// Include Messages
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Polygon.h"
#include "mower_map/MapArea.h"

// Include Service Messages
#include "mower_map/AddMowingAreaSrv.h"
#include "mower_map/ClearMapSrv.h"
#include "mower_map/ClearNavPointSrv.h"
#include "mower_map/GetDockingPointSrv.h"
#include "mower_map/GetMowingAreaSrv.h"
#include "mower_map/SetDockingPointSrv.h"
#include "mower_map/SetNavPointSrv.h"

// JSON for map storage
#include <fstream>
#include <nlohmann/json.hpp>
#include <random>
#include <string>
#include <vector>
using json = nlohmann::ordered_json;

// Monitoring
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "xbot_msgs/MapSize.h"

// RPC
#include "xbot_rpc/provider.h"

const std::string MAP_FILE = "map.json";
const std::string LEGACY_MAP_FILE = "map.bag";

// Forward declarations
void saveMapToFile();
void buildMap();

// Struct definitions for JSON serialization
struct Point {
  double x;
  double y;
};

typedef std::vector<Point> Polygon;

struct MapArea {
  std::string id;
  std::string name;
  std::string type;
  bool active;
  Polygon outline;
};

struct DockingStation {
  std::string id;
  std::string name;
  bool active;
  Point position;
  double heading;
};

struct MapData {
  std::vector<MapArea> areas;
  std::vector<DockingStation> docking_stations;

  std::vector<MapArea> getMowingAreas() {
    std::vector<MapArea> result;
    for (const auto& area : areas) {
      if (area.type == "mow") result.push_back(area);
    }
    return result;
  }

  void clear() {
    areas.clear();
    docking_stations.clear();
  }

  std::string toJsonString();
};

// JSON serialization macros
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Point, x, y)

void to_json(json& j, const MapArea& data) {
  j["id"] = data.id;
  json properties = json::object();
  if (!data.name.empty()) properties["name"] = data.name;
  properties["type"] = data.type;
  if (!data.active) properties["active"] = data.active;
  j["properties"] = properties;
  j["outline"] = data.outline;
}

void from_json(const json& j, MapArea& data) {
  j.at("id").get_to(data.id);
  const auto& properties = j.value("properties", json::object());
  data.name = properties.value("name", "");
  data.type = properties.value("type", "draft");
  data.active = properties.value("active", true);
  j.at("outline").get_to(data.outline);
}

void to_json(json& j, const DockingStation& data) {
  j["id"] = data.id;
  json properties = json::object();
  if (!data.name.empty()) properties["name"] = data.name;
  if (!data.active) properties["active"] = data.active;
  if (!properties.empty()) j["properties"] = properties;
  j["position"] = data.position;
  j["heading"] = data.heading;
}

void from_json(const json& j, DockingStation& data) {
  j.at("id").get_to(data.id);
  const auto& properties = j.value("properties", json::object());
  data.name = properties.value("name", "");
  data.active = properties.value("active", true);
  j.at("position").get_to(data.position);
  j.at("heading").get_to(data.heading);
}

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MapData, areas, docking_stations)

std::string MapData::toJsonString() {
  json json_data = *this;
  return json_data.dump(2);
}

std::string generateNanoId(size_t length = 32) {
  static const char alphabet[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
  thread_local std::mt19937 rng{std::random_device{}()};
  thread_local std::uniform_int_distribution<> dist(0, sizeof(alphabet) - 2);
  std::string id(length, '\0');
  std::generate_n(id.begin(), length, [&]() { return alphabet[dist(rng)]; });
  return id;
}

// Publishes the map as JSON string
ros::Publisher json_map_pub;

// Publishes the map as occupancy grid
ros::Publisher map_pub;

// Publishes the map as markers for rviz
ros::Publisher map_server_viz_array_pub;

// Publishes map size for heatmap generator
ros::Publisher map_size_pub;

// MapData instance - the source of truth for map data
MapData map_data;

bool show_fake_obstacle = false;
geometry_msgs::Pose fake_obstacle_pose;

// The grid map. This is built from the polygons loaded from the file.
grid_map::GridMap map;

// clang-format off
xbot_rpc::RpcProvider rpc_provider("mower_map_service", {{
  RPC_METHOD("map.replace", {
    if (!params.is_array() || params.size() != 1) {
      throw xbot_rpc::RpcException(xbot_rpc::RpcError::ERROR_INVALID_PARAMS, "Missing map parameter");
    }
    try {
      map_data = params[0];
    } catch (const std::exception& e) {
      throw xbot_rpc::RpcException(xbot_rpc::RpcError::ERROR_INVALID_PARAMS, "Invalid map: " + std::string(e.what()));
    }
    saveMapToFile();
    ROS_INFO_STREAM("Loaded " << map_data.areas.size() << " areas via RPC and saved to file");
    buildMap();
    return "Successfully stored map (" + std::to_string(map_data.areas.size()) + " areas)";
  }),
}});
// clang-format on

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
MapArea mowerMapAreaToInternal(const geometry_msgs::Polygon& area, const std::string& type, const std::string& name) {
  MapArea result;
  result.id = generateNanoId();
  result.type = type;
  result.name = name;
  result.active = true;
  result.outline = geometryPolygonToInternal(area);
  return result;
}

/**
 * Convert our internal MapArea struct to mower_map::MapArea
 */
mower_map::MapArea internalMapAreaToMower(const MapArea& area) {
  mower_map::MapArea result;
  result.name = area.name;
  result.area = internalPolygonToGeometry(area.outline);
  return result;
}

grid_map::Polygon internalPolygonToGridMap(const Polygon& poly) {
  grid_map::Polygon result;
  for (const auto& point : poly) {
    result.addVertex(grid_map::Position(point.x, point.y));
  }
  return result;
}

/**
 * Publish map to xbot_monitoring
 */
void publishMapMonitoring() {
  xbot_msgs::MapSize map_size;
  map_size.mapWidth = map.getSize().x() * map.getResolution();
  map_size.mapHeight = map.getSize().y() * map.getResolution();
  auto mapPos = map.getPosition();
  map_size.mapCenterX = mapPos.x();
  map_size.mapCenterY = mapPos.y();
  map_size_pub.publish(map_size);

  std_msgs::String json_map;
  json_map.data = map_data.toJsonString();
  json_map_pub.publish(json_map);
}

/**
 * Publish map visualizations for rviz.
 */
void visualizeAreas() {
  auto mapPos = map.getPosition();

  visualization_msgs::MarkerArray markerArray;

  for (const auto& area : map_data.areas) {
    if (!area.active) continue;
    if (area.type != "mow" && area.type != "obstacle") continue;

    std_msgs::ColorRGBA color;
    if (area.type == "mow") {
      color.g = 1.0;
    } else if (area.type == "obstacle") {
      color.r = 1.0;
    }
    color.a = 1.0;

    grid_map::Polygon p = internalPolygonToGridMap(area.outline);
    visualization_msgs::Marker marker;
    grid_map::PolygonRosConverter::toLineMarker(p, color, 0.05, 0, marker);

    marker.header.frame_id = "map";
    marker.ns = "mower_map_service";
    marker.id = markerArray.markers.size();
    marker.frame_locked = true;
    marker.pose.orientation.w = 1.0;

    markerArray.markers.push_back(marker);
  }

  // Visualize Docking Point
  if (!map_data.docking_stations.empty()) {
    const DockingStation& ds = map_data.docking_stations.front();
    geometry_msgs::Pose docking_pose;
    docking_pose.position.x = ds.position.x;
    docking_pose.position.y = ds.position.y;
    docking_pose.position.z = 0.0;

    double heading = ds.heading;
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
  for (const auto& area : map_data.areas) {
    if (!area.active) continue;
    if (area.type != "mow" && area.type != "nav" && area.type != "obstacle") continue;
    for (const auto& point : area.outline) {
      minX = std::min(minX, (float)point.x);
      maxX = std::max(maxX, (float)point.x);
      minY = std::min(minY, (float)point.y);
      maxY = std::max(maxY, (float)point.y);
    }
  }

  // Enlarge the map by 1m in all directions.
  // This guarantees that even after blurring, the map has an occupied border.
  maxX += 1.0;
  minX -= 1.0;
  maxY += 1.0;
  minY -= 1.0;

  // Check, if the map was empty. If so, we'd create a huge map. Therefore we build an empty 10x10m map instead.
  if (map_data.areas.empty()) {
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
  for (const auto& area : map_data.areas) {
    if (!area.active) continue;

    double value;
    if (area.type == "mow" || area.type == "nav") {
      value = 0.0;
    } else if (area.type == "obstacle") {
      value = 1.0;
    } else {
      continue;
    }

    grid_map::Polygon poly = internalPolygonToGridMap(area.outline);
    for (grid_map::PolygonIterator iterator(map, poly); !iterator.isPastEnd(); ++iterator) {
      const grid_map::Index index(*iterator);
      data(index[0], index[1]) = value;
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
  std::ofstream file(MAP_FILE);
  if (file.is_open()) {
    file << map_data.toJsonString();
    file.close();
    ROS_INFO("Map saved to JSON file");
  } else {
    ROS_ERROR("Failed to open JSON file for writing");
  }
}

/**
 * Load the map from a JSON file and build a map.
 */
void readMapFromFile() {
  std::ifstream json_file(MAP_FILE);
  if (json_file.is_open()) {
    try {
      json loaded_data;
      json_file >> loaded_data;
      json_file.close();

      map_data = loaded_data;

      ROS_INFO_STREAM("Loaded " << map_data.areas.size() << " areas from: " << MAP_FILE);
    } catch (const std::exception& e) {
      ROS_ERROR_STREAM("Failed to parse " << MAP_FILE << ": " << e.what());
    }
  } else {
    ROS_WARN_STREAM("Could not open map file: " << MAP_FILE);
  }
}

bool addMowingArea(mower_map::AddMowingAreaSrvRequest& req, mower_map::AddMowingAreaSrvResponse& res) {
  ROS_INFO_STREAM("Got addMowingArea call");

  map_data.areas.push_back(mowerMapAreaToInternal(req.area.area, req.isNavigationArea ? "nav" : "mow", req.area.name));
  for (const auto& obstacle : req.area.obstacles) {
    map_data.areas.push_back(mowerMapAreaToInternal(obstacle, "obstacle", ""));
  }

  saveMapToFile();
  buildMap();
  return true;
}

bool getMowingArea(mower_map::GetMowingAreaSrvRequest& req, mower_map::GetMowingAreaSrvResponse& res) {
  ROS_INFO_STREAM("Got getMowingArea call with index: " << req.index);

  auto mowing_areas = map_data.getMowingAreas();
  if (req.index >= mowing_areas.size()) {
    ROS_ERROR_STREAM("No mowing area with index: " << req.index);
    return false;
  }

  res.area = internalMapAreaToMower(mowing_areas[req.index]);

  for (const auto& area : map_data.areas) {
    if (!area.active || area.type != "obstacle") continue;
    // TODO: Check whether the obstacle overlaps with the mowing area, maybe cut it to the intersection.
    res.area.obstacles.push_back(internalPolygonToGeometry(area.outline));
  }

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

  map_data.docking_stations.clear();
  map_data.docking_stations.push_back({.id = generateNanoId(),
                                       .name = "Docking Station",
                                       .active = true,
                                       .position = {req.docking_pose.position.x, req.docking_pose.position.y},
                                       .heading = heading});

  saveMapToFile();
  buildMap();

  return true;
}

bool getDockingPoint(mower_map::GetDockingPointSrvRequest& req, mower_map::GetDockingPointSrvResponse& res) {
  ROS_INFO_STREAM("Getting Docking Point");

  if (map_data.docking_stations.empty()) {
    return false;
  }

  const DockingStation& ds = map_data.docking_stations.front();
  res.docking_pose.position.x = ds.position.x;
  res.docking_pose.position.y = ds.position.y;
  res.docking_pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, ds.heading);
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

/**
 * Helper function to convert legacy map areas from bag file
 * @param bag The rosbag to read from
 * @param topic_name The topic name to query (e.g. "mowing_areas" or "navigation_areas")
 * @param area_type The type to assign to the converted areas (e.g. "mow" or "nav")
 */
void convertLegacyAreas(rosbag::Bag& bag, const std::string& topic_name, const std::string& area_type) {
  rosbag::View view(bag, rosbag::TopicQuery(topic_name));
  for (rosbag::MessageInstance const m : view) {
    auto area = m.instantiate<mower_map::MapArea>();
    if (area) {
      // Convert main area
      MapArea main_area;
      main_area.id = generateNanoId();
      main_area.name = area->name;
      main_area.type = area_type;
      main_area.active = true;
      main_area.outline = geometryPolygonToInternal(area->area);
      map_data.areas.push_back(main_area);

      // Convert obstacles as separate areas
      for (const auto& obstacle : area->obstacles) {
        MapArea obs_area;
        obs_area.id = generateNanoId();
        obs_area.name = "";
        obs_area.type = "obstacle";
        obs_area.active = true;
        obs_area.outline = geometryPolygonToInternal(obstacle);
        map_data.areas.push_back(obs_area);
      }
    }
  }
}

void convertLegacyMapToJson() {
  // Open the legacy map file
  rosbag::Bag bag;
  try {
    bag.open(LEGACY_MAP_FILE);
  } catch (rosbag::BagIOException& e) {
    ROS_ERROR_STREAM("Error opening legacy map file for conversion: " << e.what());
    return;
  }

  // Clear the current map_data
  map_data.clear();

  // Read mowing and navigation areas
  convertLegacyAreas(bag, "mowing_areas", "mow");
  convertLegacyAreas(bag, "navigation_areas", "nav");

  // Read docking point
  {
    rosbag::View view(bag, rosbag::TopicQuery("docking_point"));
    for (rosbag::MessageInstance const m : view) {
      auto pt = m.instantiate<geometry_msgs::Pose>();
      if (pt) {
        // Convert quaternion to yaw
        tf2::Quaternion q;
        tf2::fromMsg(pt->orientation, q);
        tf2::Matrix3x3 m(q);
        double unused1, unused2, yaw;
        m.getRPY(unused1, unused2, yaw);

        // Create docking station
        DockingStation ds;
        ds.id = generateNanoId();
        ds.name = "Docking Station";
        ds.active = true;
        ds.position = {pt->position.x, pt->position.y};
        ds.heading = yaw;
        map_data.docking_stations.push_back(ds);
      }
    }
  }

  bag.close();

  // Save the converted data to JSON file
  saveMapToFile();

  ROS_INFO_STREAM("Successfully converted legacy map to JSON with "
                  << map_data.areas.size() << " areas and " << map_data.docking_stations.size() << " docking stations");
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "mower_map_service");
  ros::NodeHandle n;
  json_map_pub = n.advertise<std_msgs::String>("mower_map_service/json_map", 1, true);
  map_pub = n.advertise<nav_msgs::OccupancyGrid>("mower_map_service/map", 10, true);
  map_server_viz_array_pub = n.advertise<visualization_msgs::MarkerArray>("mower_map_service/map_viz", 10, true);
  map_size_pub = n.advertise<xbot_msgs::MapSize>("mower_map_service/map_size", 10, true);

  rpc_provider.init();

  if (!std::filesystem::exists(MAP_FILE) && std::filesystem::exists(LEGACY_MAP_FILE)) {
    ROS_INFO("Found legacy map file, converting to JSON...");
    convertLegacyMapToJson();
  } else {
    readMapFromFile();
  }

  buildMap();

  ros::ServiceServer add_area_srv = n.advertiseService("mower_map_service/add_mowing_area", addMowingArea);
  ros::ServiceServer get_area_srv = n.advertiseService("mower_map_service/get_mowing_area", getMowingArea);
  ros::ServiceServer set_docking_point_srv = n.advertiseService("mower_map_service/set_docking_point", setDockingPoint);
  ros::ServiceServer get_docking_point_srv = n.advertiseService("mower_map_service/get_docking_point", getDockingPoint);
  ros::ServiceServer set_nav_point_srv = n.advertiseService("mower_map_service/set_nav_point", setNavPoint);
  ros::ServiceServer clear_nav_point_srv = n.advertiseService("mower_map_service/clear_nav_point", clearNavPoint);
  ros::ServiceServer clear_map_srv = n.advertiseService("mower_map_service/clear_map", clearMap);

  ros::spin();
  return 0;
}
