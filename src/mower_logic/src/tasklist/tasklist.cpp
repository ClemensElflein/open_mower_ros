#include <actionlib/client/simple_action_client.h>
#include <cryptopp/cryptlib.h>
#include <cryptopp/hex.h>
#include <cryptopp/sha.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf2/LinearMath/Vector3.h>

#include "mower_logic/CheckPoint.h"
#include "mower_logic/TasklistConfig.h"
#include "mower_map/GetMowingAreaSrv.h"
#include "mower_msgs/MowPathsAction.h"
#include "slic3r_coverage_planner/PlanPath.h"

ros::ServiceClient pathClient, mapClient;

actionlib::SimpleActionClient<mower_msgs::MowPathsAction> *mowPathsClient;

dynamic_reconfigure::Server<mower_logic::TasklistConfig> *reconfigServer;
mower_logic::TasklistConfig config;

struct Task {
  int area_index;
  int start_path;
  double angle_offset;
  bool angle_offset_is_absolute;
  // TODO: Add more settings.
};

double currentMowingAngleIncrementSum = 0;
std::string currentMowingPlanDigest = "";

double area_base_angle(const mower_map::MapArea &area) {
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

mower_msgs::MowPathsGoalPtr create_mowing_plan(Task task) {
  mower_msgs::MowPathsGoalPtr goal(new mower_msgs::MowPathsGoal);

  ROS_INFO_STREAM("MowingBehavior: Creating mowing plan for area: " << task.area_index);

  // get the mowing area
  mower_map::GetMowingAreaSrv mapSrv;
  mapSrv.request.index = task.area_index;
  if (!mapClient.call(mapSrv)) {
    ROS_ERROR_STREAM("MowingBehavior: Error loading mowing area");
    return nullptr;
  }

  double angle = area_base_angle(mapSrv.response.area);

  // add mowing angle offset increment and return into the <-180, 180> range
  double mow_angle_offset = std::fmod(task.angle_offset + currentMowingAngleIncrementSum + 180, 360);
  if (mow_angle_offset < 0) mow_angle_offset += 360;
  mow_angle_offset -= 180;
  ROS_INFO_STREAM("MowingBehavior: mowing angle offset (deg): " << mow_angle_offset);
  if (task.angle_offset_is_absolute) {
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
    return nullptr;
  }

  goal->paths = pathSrv.response.paths;
  goal->start_path = task.start_path;
  goal->start_point = 0;

  // Calculate mowing plan digest from the poses
  // TODO: At this point, we need to load the checkpoint. Or maybe we'll save that as part of the task list, along with
  // other progress indicators.
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

bool handle_tasks() {
  std::vector<Task> tasks;
  Task t{
      .area_index = 0,
      .start_path = 0,
      .angle_offset = config.mow_angle_offset,
      .angle_offset_is_absolute = config.mow_angle_offset_is_absolute,
  };
  tasks.push_back(t);
  t.start_path = 1;
  t.angle_offset += 90;
  tasks.push_back(t);

  size_t remaining = tasks.size();
  for (auto task : tasks) {
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
