#include <actionlib/client/simple_action_client.h>
#include <cryptopp/cryptlib.h>
#include <cryptopp/hex.h>
#include <cryptopp/sha.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf2/LinearMath/Vector3.h>

#include <condition_variable>
#include <mutex>
#include <nlohmann/json.hpp>

#include "../../include/utils.h"
#include "mower_logic/CheckPoint.h"
#include "mower_logic/TasklistConfig.h"
#include "mower_map/GetMowingAreaSrv.h"
#include "mower_msgs/MowPathsAction.h"
#include "slic3r_coverage_planner/PlanPath.h"
#include "xbot_msgs/ActionRequest.h"
#include "xbot_msgs/ActionResponse.h"
#include "xbot_msgs/RegisterActionsSrv.h"

using json = nlohmann::json;

ros::ServiceClient pathClient, mapClient, actionRegistrationClient;

typedef actionlib::SimpleActionClient<mower_msgs::MowPathsAction> MowPathsClient;
MowPathsClient *mowPathsClient;

dynamic_reconfigure::Server<mower_logic::TasklistConfig> *reconfigServer;
mower_logic::TasklistConfig config;

ros::Publisher action_response_pub;

auto action_skip_area = xbot_msgs::create_action("mower_logic:mowing/skip_area", "Skip Area", true);
auto action_skip_path = xbot_msgs::create_action("mower_logic:mowing/skip_path", "Skip Path");
std::vector<xbot_msgs::ActionInfo> actions = {action_skip_area, action_skip_path};

struct Task {
  json params;
  bool is_last;
};

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

std::mutex mtx_tasklist;
std::condition_variable cv_tasklist_or_idx_changed;
std::condition_variable cv_save_progress;
json current_tasklist;
json *current_task_idx, *current_path, *current_point;
const int RTI_NONE = -1;
const int RTI_RELATIVE = -2;
int requested_task_idx = RTI_NONE;
int requested_task_idx_relative = 0;

void change_tasklist(std::function<void()> cb, bool cancel_current_task = true) {
  std::lock_guard<std::mutex> lk(mtx_tasklist);
  if (cancel_current_task) {
    mowPathsClient->cancelAllGoals();
  }
  cb();
  cv_tasklist_or_idx_changed.notify_all();
  cv_save_progress.notify_all();
}

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

  return tasklist;
}

void *progress_saver_thread(void *context) {
  std::unique_lock<std::mutex> lock(mtx_tasklist);
  size_t last_hash = 0;
  while (true) {
    size_t hash = std::hash<json>{}(current_tasklist);
    if (hash != last_hash) {
      std::ofstream o("tasklist.json");
      o << std::setw(4) << current_tasklist << std::endl;
      last_hash = hash;
    }

    // Wait until we get a signal or 30 seconds have passed.
    cv_save_progress.wait_for(lock, std::chrono::seconds(30));
  }
}

Task get_next_task() {
  std::unique_lock<std::mutex> lk(mtx_tasklist);
  while (ros::ok()) {
    // Find out which tasks to start next. Prefer explicitly requested task.
    int next_task_idx = requested_task_idx;
    requested_task_idx = RTI_NONE;
    if (next_task_idx < 0) {
      if (current_task_idx->is_null()) {
        // We had reached the end of the tasklist, so all we can do is wait for new commands.
        // Wake up once per second to check whether we should shut down.
        cv_tasklist_or_idx_changed.wait_for(lk, std::chrono::seconds(1));
        continue;
      } else if (next_task_idx == RTI_RELATIVE) {
        next_task_idx = (int)*current_task_idx + requested_task_idx_relative;
      } else {
        next_task_idx = (int)*current_task_idx + 1;
      }
    }

    // Check whether we're past the last tasklist item.
    const auto &tasks = current_tasklist["tasks"];
    if (next_task_idx >= tasks.size()) {
      if (current_tasklist.value("repeat", false)) {
        // Repeat mode is enabled, so wrap around.
        next_task_idx = 0;
      } else {
        // Unset the entry and restart the loop, which will take care of waiting for changes.
        *current_task_idx = nullptr;
        cv_tasklist_or_idx_changed.notify_all();
        continue;
      }
    }

    // Looks like we have a valid task, so save that.
    *current_task_idx = next_task_idx;
    cv_tasklist_or_idx_changed.notify_all();

    // Now load the parameters.
    Task task = {
        .params = tasks[next_task_idx],
        .is_last = false,
    };
    if (next_task_idx + 1 == tasks.size() && !current_tasklist.value("repeat", false)) {
      task.is_last = true;
    }
    return task;
  }
  return Task();
}

void feedbackCb(const mower_msgs::MowPathsFeedbackConstPtr &feedback) {
  std::unique_lock<std::mutex> lock(mtx_tasklist);
  bool significant_update = *current_path != feedback->current_path;
  *current_path = feedback->current_path;
  *current_point = feedback->current_point;
  if (significant_update) {
    cv_save_progress.notify_all();
  }
}

void handle_tasks() {
  while (ros::ok()) {
    auto task = get_next_task();
    current_tasklist["progress"]["current_path"] = 0;
    current_tasklist["progress"]["current_point"] = 0;
    cv_save_progress.notify_all();

    if (!ros::ok()) {
      return;
    }

    auto goal = create_mowing_plan(task.params);
    if (goal == nullptr) {
      ROS_ERROR_STREAM("Could not create mowing plan");
      continue;
    }

    // We have a plan, execute it
    goal->expect_more_goals = !task.is_last;
    ROS_INFO_STREAM("MowingBehavior: Executing mowing plan");
    mowPathsClient->sendGoal(*goal,
                             MowPathsClient::SimpleDoneCallback(),    // empty
                             MowPathsClient::SimpleActiveCallback(),  // empty
                             feedbackCb);
    mowPathsClient->waitForResult();
    auto result = mowPathsClient->getState();
    if (result != actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_ERROR_STREAM("Task finished with state " << result.toString());
      continue;
    }
  }
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

void add_default_progress(json &j) {
  j.emplace("progress", json::object());
  auto &progress = j["progress"];
  progress.emplace("current_path", 0);
  progress.emplace("current_point", 0);
  progress.emplace("current_task", 0);

  // Update the pointers.
  current_task_idx = &progress["current_task"];
  current_path = &progress["current_path"];
  current_point = &progress["current_point"];
}

void action_received(const xbot_msgs::ActionRequest::ConstPtr &request) {
  xbot_msgs::ActionResponse response;
  response.action_id = request->action_id;
  response.request_id = request->request_id;

  if (request->action_id == "tasklist/set_tasklist") {
    try {
      auto new_tasklist = json::parse(request->payload);
      add_default_progress(new_tasklist);
      ROS_INFO_STREAM("New tasklist: " << new_tasklist.dump());
      change_tasklist([new_tasklist] {
        current_tasklist = new_tasklist;
        requested_task_idx = 0;
      });
    } catch (const json::parse_error &e) {
      ROS_ERROR_STREAM("Could not parse JSON in set_tasklist:" << e.what());
      return;
    }
  } else if (request->action_id == "tasklist/set_default_tasklist") {
    auto new_tasklist = create_default_tasklist();
    add_default_progress(new_tasklist);
    ROS_INFO_STREAM("New (default) tasklist: " << new_tasklist.dump());
    change_tasklist([new_tasklist] {
      current_tasklist = new_tasklist;
      requested_task_idx = 0;
    });
  } else if (request->action_id == "tasklist/restart") {
    change_tasklist([] { requested_task_idx = 0; });
  } else if (request->action_id == "tasklist/next_task" || request->action_id == action_skip_area.action_id) {
    change_tasklist([] {
      requested_task_idx = RTI_RELATIVE;
      requested_task_idx_relative = 1;
    });
  } else if (request->action_id == action_skip_path.action_id) {
    // TODO: Implement skipping a path.
  } else if (request->action_id == "tasklist/enable_repeat") {
    change_tasklist([] { current_tasklist["repeat"] = true; }, false);
  } else {
    return;
  }

  action_response_pub.publish(response);
}

void registerActions(std::string prefix, const std::vector<xbot_msgs::ActionInfo> &actions) {
  xbot_msgs::RegisterActionsSrv srv;
  srv.request.node_prefix = prefix;
  srv.request.actions = actions;

  ros::Rate retry_delay(1);
  actionRegistrationClient.waitForExistence(ros::Duration(10));
  for (int i = 0; i < 10; i++) {
    if (actionRegistrationClient.call(srv)) {
      ROS_INFO_STREAM("successfully registered actions for " << prefix);
      break;
    }
    ROS_ERROR_STREAM("Error registering actions for " << prefix << ". Retrying.");
    retry_delay.sleep();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "mower_logic");

  current_tasklist = {
      {"tasks", json::array()},
  };
  add_default_progress(current_tasklist);

  ros::NodeHandle n;
  ros::NodeHandle paramNh("~");

  dynamic_reconfigure::Server<mower_logic::TasklistConfig> reconfig_server(paramNh);
  reconfig_server.setCallback(reconfigureCB);

  pthread_t progress_saver_thread_handle;
  pthread_create(&progress_saver_thread_handle, nullptr, &progress_saver_thread, nullptr);

  pathClient = n.serviceClient<slic3r_coverage_planner::PlanPath>("slic3r_coverage_planner/plan_path");
  mapClient = n.serviceClient<mower_map::GetMowingAreaSrv>("mower_map_service/get_mowing_area");
  actionRegistrationClient = n.serviceClient<xbot_msgs::RegisterActionsSrv>("xbot/register_actions");
  registerActions("tasklist", actions);

  ros::Subscriber action = n.subscribe("xbot/action", 0, action_received, ros::TransportHints().tcpNoDelay(true));
  action_response_pub = n.advertise<xbot_msgs::ActionResponse>("xbot/action_response", 100);

  ros::AsyncSpinner spinner(1);
  spinner.start();

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

  mowPathsClient = new MowPathsClient("mower_logic/mow_paths");
  if (!mowPathsClient->waitForServer(ros::Duration(60.0, 0.0))) {
    ROS_ERROR("Mow paths action server not found.");
    return 2;
  }

  handle_tasks();
  return 0;
}
