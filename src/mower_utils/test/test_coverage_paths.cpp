// Copyright (c) 2026 OpenMower contributors. All rights reserved.
//
// Rostest that drives the real production C++ pipeline (mower_logic ->
// MowingBehavior -> slic3r_coverage_planner) for every mowing area in a
// fixture map and validates each captured ExePath goal against the
// OccupancyGrid published by mower_map_service.
//
// Fake mbf action servers (move_base_flex/exe_path and move_base_flex/move_base)
// hosted inside the test executable accept goals and immediately succeed,
// letting the state machine march through every path of every area without
// needing a real controller or simulator drive loop.

#include <actionlib/server/simple_action_server.h>
#include <ftc_local_planner/PlannerGetProgress.h>
#include <gtest/gtest.h>
#include <mbf_msgs/ExePathAction.h>
#include <mbf_msgs/MoveBaseAction.h>
#include <mower_msgs/ESCStatus.h>
#include <mower_msgs/Emergency.h>
#include <mower_msgs/EmergencyStopSrv.h>
#include <mower_msgs/HighLevelControlSrv.h>
#include <mower_msgs/HighLevelStatus.h>
#include <mower_msgs/MowerControlSrv.h>
#include <mower_msgs/Power.h>
#include <mower_msgs/Status.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <xbot_positioning/GPSControlSrv.h>
#include <xbot_positioning/SetPoseSrv.h>
#include <xbot_rpc/RpcError.h>
#include <xbot_rpc/RpcRequest.h>
#include <xbot_rpc/RpcResponse.h>

#include <boost/bind.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

using ExePathServer = actionlib::SimpleActionServer<mbf_msgs::ExePathAction>;
using MoveBaseServer = actionlib::SimpleActionServer<mbf_msgs::MoveBaseAction>;

std::string slurp(const std::string& path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("cannot open fixture: " + path);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

// Counts type=="mow" entries in the JSON fixture using a string scan.
// We deliberately avoid pulling nlohmann::json here because it would force
// us to add it to find_package; the fixture format is stable and tiny.
int countMowAreas(const std::string& json) {
  int count = 0;
  size_t pos = 0;
  const std::string needle = "\"type\":\"mow\"";
  while ((pos = json.find(needle, pos)) != std::string::npos) {
    ++count;
    pos += needle.size();
  }
  return count;
}

class CoveragePathsTest : public ::testing::Test {
 protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_{"~"};

  std::unique_ptr<ExePathServer> exePathServer_;
  std::unique_ptr<MoveBaseServer> moveBaseServer_;

  ros::Subscriber statusSub_;
  ros::Subscriber mapSub_;
  ros::Publisher rpcReqPub_;
  ros::Subscriber rpcRespSub_;
  ros::Subscriber rpcErrSub_;
  ros::Publisher actionPub_;
  ros::ServiceClient highLevelCtrl_;

  // Stubs that mimic mower_comms_v2 + xbot_positioning + FTCPlanner so that
  // mower_logic's startup waits all unblock without a real controller stack.
  ros::Publisher emergencyPub_, statusPub_, powerPub_, leftEscPub_, rightEscPub_;
  ros::ServiceServer emergencySrv_, mowEnabledSrv_, gpsCtrlSrv_, setPoseSrv_, ftcProgressSrv_;
  ros::Timer stubPubTimer_;

  std::mutex m_;
  nav_msgs::OccupancyGrid::ConstPtr grid_;
  int currentArea_ = -1;
  uint8_t lastStatusState_ = 0xFF;
  std::map<int, std::vector<nav_msgs::Path>> capturedByArea_;
  std::set<std::string> rpcAcked_;

  std::string fixturePath_;
  std::string fixtureJson_;
  int expectedAreas_ = 0;
  int lethalThreshold_ = 50;
  double perAreaTimeoutS_ = 60.0;
  double rpcWaitS_ = 30.0;

  void onExePath(const mbf_msgs::ExePathGoalConstPtr& goal) {
    {
      std::lock_guard<std::mutex> lk(m_);
      capturedByArea_[currentArea_].push_back(goal->path);
      ROS_INFO("[fake-exe] area=%d captured path with %zu poses (total in area: %zu)", currentArea_,
               goal->path.poses.size(), capturedByArea_[currentArea_].size());
    }
    // Keep the goal ACTIVE for one MowingBehavior poll cycle (~100ms +
    // initial 1s sleep before the first poll). MowingBehavior treats
    // ExePath SUCCEEDED with PlannerGetProgress < poses.size()-5 as an MBF
    // failure and goes into PAUSE; we keep it active long enough for it to
    // pick up the "fully consumed" progress index our FTC stub reports.
    ros::Duration(1.5).sleep();
    mbf_msgs::ExePathResult result;
    result.outcome = mbf_msgs::ExePathResult::SUCCESS;
    result.message = "fake server: success after fake drive";
    exePathServer_->setSucceeded(result);
  }

  void onMoveBase(const mbf_msgs::MoveBaseGoalConstPtr& /*goal*/) {
    mbf_msgs::MoveBaseResult result;
    result.outcome = mbf_msgs::MoveBaseResult::SUCCESS;
    result.message = "fake server: instant success";
    moveBaseServer_->setSucceeded(result);
  }

  void onStatus(const mower_msgs::HighLevelStatus::ConstPtr& msg) {
    std::lock_guard<std::mutex> lk(m_);
    uint8_t state5 = msg->state & 0x1F;
    if (msg->current_area != currentArea_ || state5 != lastStatusState_) {
      ROS_INFO("[status] state=%u sub=%u area=%d path=%d idx=%d (was area=%d state=%u)", state5,
               msg->state >> mower_msgs::HighLevelStatus::SUBSTATE_SHIFT, msg->current_area, msg->current_path,
               msg->current_path_index, currentArea_, lastStatusState_);
      currentArea_ = msg->current_area;
      lastStatusState_ = state5;
    }
  }

  void onMap(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    std::lock_guard<std::mutex> lk(m_);
    grid_ = msg;
    ROS_INFO("[map] occupancy grid: %ux%u @ %.3f m/cell, origin=(%.2f,%.2f)", msg->info.width, msg->info.height,
             msg->info.resolution, msg->info.origin.position.x, msg->info.origin.position.y);
  }

  void onRpcResponse(const xbot_rpc::RpcResponse::ConstPtr& msg) {
    std::lock_guard<std::mutex> lk(m_);
    rpcAcked_.insert(msg->id);
    ROS_INFO("[rpc] response id=%s result=%s", msg->id.c_str(), msg->result.c_str());
  }

  void onRpcError(const xbot_rpc::RpcError::ConstPtr& msg) {
    std::lock_guard<std::mutex> lk(m_);
    rpcAcked_.insert(msg->id);
    ROS_ERROR("[rpc] error id=%s", msg->id.c_str());
  }

  bool waitFor(std::function<bool()> pred, double timeout_s) {
    ros::Time deadline = ros::Time::now() + ros::Duration(timeout_s);
    ros::Rate r(20);
    while (ros::ok() && ros::Time::now() < deadline) {
      if (pred()) return true;
      r.sleep();
    }
    return pred();
  }

  void SetUp() override {
    ASSERT_TRUE(pnh_.getParam("fixture_path", fixturePath_));
    pnh_.param("lethal_threshold", lethalThreshold_, 50);
    pnh_.param("per_area_timeout_s", perAreaTimeoutS_, 60.0);
    pnh_.param("rpc_wait_s", rpcWaitS_, 30.0);

    fixtureJson_ = slurp(fixturePath_);
    expectedAreas_ = countMowAreas(fixtureJson_);
    ASSERT_GT(expectedAreas_, 0) << "fixture has no mowing areas";
    ROS_INFO("Fixture: %s (%d mowing areas)", fixturePath_.c_str(), expectedAreas_);

    // Advertise fake action servers BEFORE we expect mower_logic's clients to
    // connect. mower_logic blocks up to 60s for these in main().
    exePathServer_ = std::make_unique<ExePathServer>(nh_, "move_base_flex/exe_path",
                                                     boost::bind(&CoveragePathsTest::onExePath, this, _1), false);
    exePathServer_->start();
    moveBaseServer_ = std::make_unique<MoveBaseServer>(nh_, "move_base_flex/move_base",
                                                       boost::bind(&CoveragePathsTest::onMoveBase, this, _1), false);
    moveBaseServer_->start();
    ROS_INFO("Fake mbf action servers advertised");

    // Stub services normally provided by xbot_positioning, mower_comms,
    // and FTCPlanner. mower_logic blocks on all of these at startup.
    boost::function<bool(mower_msgs::EmergencyStopSrv::Request&, mower_msgs::EmergencyStopSrv::Response&)> emergencyCb =
        [](auto&, auto&) { return true; };
    emergencySrv_ = nh_.advertiseService("ll/_service/emergency", emergencyCb);

    boost::function<bool(mower_msgs::MowerControlSrv::Request&, mower_msgs::MowerControlSrv::Response&)> mowCb =
        [](auto&, auto&) { return true; };
    mowEnabledSrv_ = nh_.advertiseService("ll/_service/mow_enabled", mowCb);

    boost::function<bool(xbot_positioning::GPSControlSrv::Request&, xbot_positioning::GPSControlSrv::Response&)> gpsCb =
        [](auto&, auto&) { return true; };
    gpsCtrlSrv_ = nh_.advertiseService("xbot_positioning/set_gps_state", gpsCb);

    boost::function<bool(xbot_positioning::SetPoseSrv::Request&, xbot_positioning::SetPoseSrv::Response&)> poseCb =
        [](auto&, auto&) { return true; };
    setPoseSrv_ = nh_.advertiseService("xbot_positioning/set_robot_pose", poseCb);

    boost::function<bool(ftc_local_planner::PlannerGetProgress::Request&,
                         ftc_local_planner::PlannerGetProgress::Response&)>
        ftcCb = [](auto&, auto& res) {
          // Report a very large progress index so MowingBehavior's
          // "(poses.size() - currentMowingPathIndex) < 5" check passes
          // immediately on SUCCEEDED, advancing to the next path/area
          // instead of treating the fast fake drive as an MBF failure.
          res.index = 1000000;
          return true;
        };
    ftcProgressSrv_ = nh_.advertiseService("/move_base_flex/FTCPlanner/planner_get_progress", ftcCb);

    // Stub publishers + 5 Hz heartbeat timer so mower_logic's StateSubscribers
    // see fresh messages on /ll/emergency, /ll/mower_status, /ll/power, and
    // both ESC status topics. All values are healthy/idle defaults.
    emergencyPub_ = nh_.advertise<mower_msgs::Emergency>("/ll/emergency", 1, true);
    statusPub_ = nh_.advertise<mower_msgs::Status>("/ll/mower_status", 1, true);
    powerPub_ = nh_.advertise<mower_msgs::Power>("/ll/power", 1, true);
    leftEscPub_ = nh_.advertise<mower_msgs::ESCStatus>("/ll/diff_drive/left_esc_status", 1, true);
    rightEscPub_ = nh_.advertise<mower_msgs::ESCStatus>("/ll/diff_drive/right_esc_status", 1, true);
    stubPubTimer_ = nh_.createTimer(ros::Duration(0.2), [this](const ros::TimerEvent&) {
      ros::Time now = ros::Time::now();
      mower_msgs::Emergency em;
      em.stamp = now;
      em.active_emergency = false;
      em.latched_emergency = false;
      emergencyPub_.publish(em);

      mower_msgs::Status st;
      st.stamp = now;
      st.mower_status = mower_msgs::Status::MOWER_STATUS_OK;
      st.raspberry_pi_power = true;
      st.esc_power = true;
      st.is_charging = false;
      st.mower_esc_status = mower_msgs::ESCStatus::ESC_STATUS_OK;
      statusPub_.publish(st);

      mower_msgs::Power pw;
      pw.stamp = now;
      pw.v_battery = 30.0;
      pw.v_charge = 0.0;
      pw.charge_current = 0.0;
      pw.charger_enabled = false;
      pw.charger_status = "DISCHARGING";
      powerPub_.publish(pw);

      mower_msgs::ESCStatus esc;
      esc.status = mower_msgs::ESCStatus::ESC_STATUS_OK;
      esc.temperature_pcb = 30.0;
      esc.temperature_motor = 30.0;
      leftEscPub_.publish(esc);
      rightEscPub_.publish(esc);
    });
    ROS_INFO("Stub services + heartbeat publishers up");

    // Topics
    statusSub_ = nh_.subscribe("/mower_logic/current_state", 10, &CoveragePathsTest::onStatus, this);
    mapSub_ = nh_.subscribe("/mower_map_service/map", 1, &CoveragePathsTest::onMap, this);
    rpcRespSub_ = nh_.subscribe("/xbot/rpc/response", 10, &CoveragePathsTest::onRpcResponse, this);
    rpcErrSub_ = nh_.subscribe("/xbot/rpc/error", 10, &CoveragePathsTest::onRpcError, this);
    rpcReqPub_ = nh_.advertise<xbot_rpc::RpcRequest>("/xbot/rpc/request", 1, true);
    actionPub_ = nh_.advertise<std_msgs::String>("/xbot/action", 5, false);

    // Wait for map service RPC to be subscribed to /xbot/rpc/request, then
    // inject the fixture map via map.replace.
    ASSERT_TRUE(waitFor([&] { return rpcReqPub_.getNumSubscribers() > 0; }, rpcWaitS_))
        << "no subscriber on /xbot/rpc/request (mower_map_service not up?)";
    ros::Duration(0.3).sleep();  // settle

    // Build [<fixture_obj>] as the params array.
    std::string params = "[" + fixtureJson_ + "]";
    xbot_rpc::RpcRequest req;
    req.method = "map.replace";
    req.params = params;
    req.id = "coverage-test-" + std::to_string(ros::Time::now().toNSec());
    rpcReqPub_.publish(req);
    ROS_INFO("Published map.replace (id=%s, %zu bytes params)", req.id.c_str(), params.size());

    ASSERT_TRUE(waitFor(
        [&] {
          std::lock_guard<std::mutex> lk(m_);
          return rpcAcked_.count(req.id) > 0;
        },
        rpcWaitS_))
        << "no rpc response/error for map.replace within " << rpcWaitS_ << "s";

    // After RPC succeeds, mower_map_service has already called buildMap() and
    // republished the latched grid. Wait for our subscriber to hold the
    // post-replace grid (distinguished from the initial 10x10m default by
    // size). buildMap inflates the bbox by 1m on each side; the user fixture
    // spans roughly 13m x 14m, so the post-replace grid is much larger than
    // the 200x200 default.
    ASSERT_TRUE(waitFor(
        [&] {
          std::lock_guard<std::mutex> lk(m_);
          return grid_ != nullptr && (grid_->info.width > 220 || grid_->info.height > 220);
        },
        30.0))
        << "post-replace occupancy grid not received within 30s";

    // Wait for high-level control service.
    highLevelCtrl_ = nh_.serviceClient<mower_msgs::HighLevelControlSrv>("/mower_service/high_level_control");
    ASSERT_TRUE(highLevelCtrl_.waitForExistence(ros::Duration(60.0))) << "high_level_control service never appeared";
  }

  // Returns the cell value at world (x,y), or -1 if outside the grid.
  int lookupCell(double x, double y) const {
    if (!grid_) return -1;
    const auto& info = grid_->info;
    double dx = x - info.origin.position.x;
    double dy = y - info.origin.position.y;
    int cx = static_cast<int>(std::floor(dx / info.resolution));
    int cy = static_cast<int>(std::floor(dy / info.resolution));
    if (cx < 0 || cy < 0 || cx >= static_cast<int>(info.width) || cy >= static_cast<int>(info.height)) {
      return -1;
    }
    return static_cast<int>(grid_->data[cy * info.width + cx]);
  }
};

TEST_F(CoveragePathsTest, AllMowingAreasAreWellCovered) {
  // Wait until mower_logic has actually entered IDLE — until then the
  // high_level_control callback would drop COMMAND_START because
  // currentBehavior is still null.
  ASSERT_TRUE(waitFor(
      [&] {
        std::lock_guard<std::mutex> lk(m_);
        return lastStatusState_ == mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_IDLE;
      },
      90.0))
      << "mower_logic never reported HIGH_LEVEL_STATE_IDLE";

  mower_msgs::HighLevelControlSrv srv;
  srv.request.command = mower_msgs::HighLevelControlSrvRequest::COMMAND_START;
  ASSERT_TRUE(highLevelCtrl_.call(srv)) << "high_level_control(START) failed";
  ROS_INFO("Sent COMMAND_START; waiting for areas to be processed...");

  // Drive through all areas. When the per-area watchdog fires, publish
  // skip_area to bump mower_logic onto the next one.
  std::set<int> seenAreas;
  ros::Time deadline = ros::Time::now() + ros::Duration(perAreaTimeoutS_ * (expectedAreas_ + 2));
  int lastSeen = -1;
  ros::Time lastChange = ros::Time::now();

  while (ros::ok() && ros::Time::now() < deadline) {
    int area;
    size_t totalAreasCaptured;
    {
      std::lock_guard<std::mutex> lk(m_);
      area = currentArea_;
      totalAreasCaptured = capturedByArea_.size();
    }
    if (area >= 0) seenAreas.insert(area);
    if (area != lastSeen) {
      lastSeen = area;
      lastChange = ros::Time::now();
    }
    if (totalAreasCaptured >= static_cast<size_t>(expectedAreas_) &&
        (ros::Time::now() - lastChange).toSec() > perAreaTimeoutS_) {
      ROS_INFO("All %d areas captured at least once; stopping.", expectedAreas_);
      break;
    }
    if ((ros::Time::now() - lastChange).toSec() > perAreaTimeoutS_) {
      ROS_WARN("No area progress for %.0fs (current=%d). Publishing skip_area.", perAreaTimeoutS_, area);
      std_msgs::String s;
      s.data = "mower_logic:mowing/skip_area";
      actionPub_.publish(s);
      lastChange = ros::Time::now();
    }
    ros::Duration(0.5).sleep();
  }

  // Dump captured paths to JSON for visualization.
  {
    std::lock_guard<std::mutex> lk(m_);
    std::ofstream dump("/tmp/captured_paths.json");
    if (dump.is_open()) {
      dump << "{\n";
      bool firstArea = true;
      for (const auto& kv : capturedByArea_) {
        if (!firstArea) dump << ",\n";
        firstArea = false;
        dump << "  \"" << kv.first << "\": [\n";
        for (size_t pi = 0; pi < kv.second.size(); ++pi) {
          dump << "    [";
          for (size_t qi = 0; qi < kv.second[pi].poses.size(); ++qi) {
            if (qi > 0) dump << ",";
            dump << "[" << kv.second[pi].poses[qi].pose.position.x << "," << kv.second[pi].poses[qi].pose.position.y
                 << "]";
          }
          dump << "]" << (pi + 1 < kv.second.size() ? "," : "") << "\n";
        }
        dump << "  ]";
      }
      dump << "\n}\n";
      dump.close();
      ROS_INFO("Dumped captured paths to /tmp/captured_paths.json");
    }
  }

  // Validate: every captured path must lie in free cells.
  std::vector<std::string> failures;
  std::lock_guard<std::mutex> lk(m_);

  ASSERT_TRUE(grid_ != nullptr);

  for (const auto& kv : capturedByArea_) {
    int a = kv.first;
    const auto& paths = kv.second;
    if (a < 0) continue;  // -1 entries are pre-area captures (e.g. undock)

    int totalPoses = 0;
    int badPoses = 0;
    int firstBadPathIdx = -1, firstBadPoseIdx = -1;
    int worstCell = 0;
    double firstBadX = 0, firstBadY = 0;

    for (size_t pi = 0; pi < paths.size(); ++pi) {
      for (size_t qi = 0; qi < paths[pi].poses.size(); ++qi) {
        ++totalPoses;
        double x = paths[pi].poses[qi].pose.position.x;
        double y = paths[pi].poses[qi].pose.position.y;
        int v = lookupCell(x, y);
        bool bad = (v < 0) || (v >= lethalThreshold_);
        if (bad) {
          if (firstBadPathIdx < 0) {
            firstBadPathIdx = pi;
            firstBadPoseIdx = qi;
            firstBadX = x;
            firstBadY = y;
          }
          if (v > worstCell) worstCell = v;
          ++badPoses;
        }
      }
    }

    if (badPoses > 0) {
      std::ostringstream os;
      os << "area[" << a << "]: " << badPoses << "/" << totalPoses << " poses in obstacle (first at path["
         << firstBadPathIdx << "] pose[" << firstBadPoseIdx << "] = (" << firstBadX << ", " << firstBadY
         << "), worst cell=" << worstCell << ", paths_in_area=" << paths.size() << ")";
      failures.push_back(os.str());
    } else {
      ROS_INFO("area[%d]: OK (%zu paths, %d poses, all in free space)", a, paths.size(), totalPoses);
    }
  }

  // Areas we never saw at all are also failures.
  for (int a = 0; a < expectedAreas_; ++a) {
    if (capturedByArea_.find(a) == capturedByArea_.end()) {
      std::ostringstream os;
      os << "area[" << a << "]: no paths captured (never visited)";
      failures.push_back(os.str());
    }
  }

  for (const auto& f : failures) {
    ADD_FAILURE() << f;
  }
  ASSERT_TRUE(failures.empty()) << failures.size() << " area(s) failed coverage validation";
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_coverage_paths");
  testing::InitGoogleTest(&argc, argv);
  ros::AsyncSpinner spinner(2);
  spinner.start();
  int rc = RUN_ALL_TESTS();
  spinner.stop();
  return rc;
}
