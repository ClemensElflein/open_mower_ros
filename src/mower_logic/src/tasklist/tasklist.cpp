#include <actionlib/client/simple_action_client.h>
#include <cryptopp/cryptlib.h>
#include <cryptopp/hex.h>
#include <cryptopp/sha.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf2/LinearMath/Vector3.h>

#include <nlohmann/json.hpp>

#include "mower_logic/CheckPoint.h"
#include "mower_logic/TasklistConfig.h"
#include "mower_map/GetMowingAreaSrv.h"
#include "mower_msgs/MowPathsAction.h"
#include "slic3r_coverage_planner/PlanPath.h"

using json = nlohmann::json;

ros::ServiceClient pathClient, mapClient;

actionlib::SimpleActionClient<mower_msgs::MowPathsAction> *mowPathsClient;

dynamic_reconfigure::Server<mower_logic::TasklistConfig> *reconfigServer;
mower_logic::TasklistConfig config;

struct TaskConfig {
  int outline_count;
  int outline_overlap_count;
  double outline_offset;

  double angle;

  bool skip_area_outline;
  bool skip_obstacle_outlines;
  bool skip_fill;
};

double currentMowingAngleIncrementSum = 0;
std::string currentMowingPlanDigest = "";

template <typename T>
T resolve_config_value(const char *key, const json &task, const json &area_config, const T fallback) {
  auto it = task.find(key);
  if (it != task.end()) {
    return *it;
  }
  it = area_config.find(key);
  if (it != area_config.end()) {
    return *it;
  }
  return fallback;
}

double area_base_angle(const mower_map::MapArea &area) {
  // TODO: Check global config (e.g. +90° for all areas).
  // TODO: How about running this after recording and persisting it (allow to edit later)?
  auto points = area.area.points;
  if (points.size() >= 2) {
    tf2::Vector3 first(points[0].x, points[0].y, 0);
    for (auto point : points) {
      tf2::Vector3 second(point.x, point.y, 0);
      auto diff = second - first;
      if (diff.length() > 2.0) {
        // we have found a point that has a distance of > 1 m, calculate the angle
        return atan2(diff.y(), diff.x());
      }
    }
  }
  return 0;
}

TaskConfig determine_task_config(const json &task, const json &area_config, const mower_map::MapArea &area) {
  TaskConfig tc;

  tc.outline_count = resolve_config_value("outline_count", task, area_config, config.outline_count);
  tc.outline_overlap_count =
      resolve_config_value("outline_overlap_count", task, area_config, config.outline_overlap_count);
  tc.outline_offset = resolve_config_value("outline_offset", task, area_config, config.outline_offset);

  if (task.find("angle") != task.end()) {
    tc.angle = (double)task["angle"] * (M_PI / 180.0);
    if (!task.value("angle_is_absolute", false)) {
      tc.angle += area_base_angle(area);
    }
  } else {
    tc.angle = area_base_angle(area);
  }
  // TODO: Normalize angle.

  tc.skip_area_outline = task.value("skip_area_outline", false);
  tc.skip_obstacle_outlines = task.value("skip_obstacle_outlines", false);
  tc.skip_fill = task.value("skip_fill", false);

  return tc;
}

mower_msgs::MowPathsGoalPtr create_mowing_plan(const json &task) {
  mower_msgs::MowPathsGoalPtr goal(new mower_msgs::MowPathsGoal);

  const size_t area_index = task["area"];
  ROS_INFO_STREAM("MowingBehavior: Creating mowing plan for area: " << area_index);

  // get the mowing area
  mower_map::GetMowingAreaSrv mapSrv;
  mapSrv.request.index = area_index;
  if (!mapClient.call(mapSrv)) {
    ROS_ERROR_STREAM("MowingBehavior: Error loading mowing area");
    return nullptr;
  }

  // FIXME: This should be loaded from somewhere, ideally stored within the map or next to it.
  const json &area_config = json::parse(R"(
    {
      "outline_count": 3
    }
  )");

  const TaskConfig task_config = determine_task_config(task, area_config, mapSrv.response.area);

  // calculate coverage
  slic3r_coverage_planner::PlanPath pathSrv;
  pathSrv.request.angle = task_config.angle;
  pathSrv.request.outline_count = task_config.outline_count;
  pathSrv.request.outline_overlap_count = task_config.outline_overlap_count;
  pathSrv.request.outline = mapSrv.response.area.area;
  pathSrv.request.holes = mapSrv.response.area.obstacles;
  pathSrv.request.fill_type = slic3r_coverage_planner::PlanPathRequest::FILL_LINEAR;
  pathSrv.request.outer_offset = task_config.outline_offset;
  pathSrv.request.distance = config.tool_width;
  pathSrv.request.skip_area_outline = task_config.skip_area_outline;
  pathSrv.request.skip_obstacle_outlines = task_config.skip_obstacle_outlines;
  pathSrv.request.skip_fill = task_config.skip_fill;
  if (!pathClient.call(pathSrv)) {
    ROS_ERROR_STREAM("MowingBehavior: Error during coverage planning");
    return nullptr;
  }

  goal->paths = pathSrv.response.paths;
  goal->start_path = 0;
  goal->start_point = 0;

  // Calculate mowing plan digest from the poses
  // TODO: At this point, we need to load the checkpoint. Or maybe we'll save that as part of the task list, along
  // with other progress indicators.
  // TODO: move to slic3r_coverage_planner
  CryptoPP::SHA256 hash;
  byte digest[CryptoPP::SHA256::DIGESTSIZE];
  for (const auto &path : goal->paths) {
    for (const auto &pose_stamped : path.path.poses) {
      hash.Update(reinterpret_cast<const byte *>(&pose_stamped.pose), sizeof(geometry_msgs::Pose));
    }
  }
  hash.Final((byte *)&digest[0]);
  CryptoPP::HexEncoder encoder;
  std::string mowingPlanDigest = "";
  encoder.Attach(new CryptoPP::StringSink(mowingPlanDigest));
  encoder.Put(digest, sizeof(digest));
  encoder.MessageEnd();

  // Proceed to checkpoint?
  if (mowingPlanDigest == currentMowingPlanDigest) {
    ROS_INFO_STREAM("MowingBehavior: Advancing to checkpoint, path: " << goal->start_path
                                                                      << " index: " << goal->start_point);
  } else {
    ROS_INFO_STREAM("MowingBehavior: Ignoring checkpoint for plan ("
                    << currentMowingPlanDigest << ") current mowing plan is (" << mowingPlanDigest << ")");
    // Plan has changed so must restart the area
    currentMowingPlanDigest = mowingPlanDigest;
  }

  return std::move(goal);
}

json create_default_tasklist() {
  json tasklist = {
      {"tasks", json::array()},
  };

  // Since there is no way to determine the number of mowing areas,
  // we have to request areas until we get an error.
  for (size_t i = 0;; i++) {
    mower_map::GetMowingAreaSrv mapSrv;
    mapSrv.request.index = i;
    if (!mapClient.call(mapSrv)) {
      break;
    }
    tasklist["tasks"].push_back(json({
        {"title", "area " + std::to_string(i)},
        {"area", i},
    }));
  }

  return std::move(tasklist);
}

bool handle_tasks() {
  // FIXME
  const json &tasklist = json::parse(R"(
        {
            "tasks": [
                {
                    "area": 0,
                    "angle": 0,
                    "angle_is_absolute": true
                },
                {
                    "area": 0,
                    "angle": 90,
                    "angle_is_absolute": true,
                    "skip_area_outline": true,
                    "skip_obstacle_outlines": true
                }
            ]
        }
    )");

  size_t remaining = tasklist["tasks"].size();
  for (auto task : tasklist["tasks"]) {
    auto goal = create_mowing_plan(task);
    if (goal == nullptr) {
      ROS_INFO_STREAM("MowingBehavior: Could not create mowing plan, docking");
      // Start again from first area next time.
      // reset();
      // We cannot create a plan, so we're probably done. Go to docking station
      return false;
    }

    // We have a plan, execute it
    goal->expect_more_goals = --remaining > 0;
    ROS_INFO_STREAM("MowingBehavior: Executing mowing plan");
    auto result = mowPathsClient->sendGoalAndWait(*goal);
    if (result != actionlib::SimpleClientGoalState::SUCCEEDED) {
      return false;
    }
  }
  return true;
}

/* // FIXME
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
*/

void reconfigureCB(mower_logic::TasklistConfig &c, uint32_t level) {
  ROS_INFO_STREAM("Setting new Tasklist Config");
  config = c;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "mower_logic");

  ros::NodeHandle n;
  ros::NodeHandle paramNh("~");

  dynamic_reconfigure::Server<mower_logic::TasklistConfig> reconfig_server(paramNh);
  reconfig_server.setCallback(reconfigureCB);

  pathClient = n.serviceClient<slic3r_coverage_planner::PlanPath>("slic3r_coverage_planner/plan_path");
  mapClient = n.serviceClient<mower_map::GetMowingAreaSrv>("mower_map_service/get_mowing_area");

  ROS_INFO("Waiting for path server");
  if (!pathClient.waitForExistence(ros::Duration(60.0, 0.0))) {
    ROS_ERROR("Path service not found.");
    return 1;
  }

  ROS_INFO("Waiting for map server");
  if (!mapClient.waitForExistence(ros::Duration(60.0, 0.0))) {
    ROS_ERROR("Map server service not found.");
    return 2;
  }

  mowPathsClient = new actionlib::SimpleActionClient<mower_msgs::MowPathsAction>("mower_logic/mow_paths");
  if (!mowPathsClient->waitForServer(ros::Duration(60.0, 0.0))) {
    ROS_ERROR("Mow paths action server not found.");
    return 2;
  }

  handle_tasks();
  ros::spin();

  return 0;
}
