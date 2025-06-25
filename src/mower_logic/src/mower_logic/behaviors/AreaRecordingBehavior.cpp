// Created by Clemens Elflein on 2/21/22.
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
#include "AreaRecordingBehavior.h"

extern ros::ServiceClient dockingPointClient;
extern ros::ServiceClient emergencyClient;
extern actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction> *mbfClient;
extern actionlib::SimpleActionClient<mbf_msgs::ExePathAction> *mbfClientExePath;
extern ros::NodeHandle *n;
extern void registerActions(std::string prefix, const std::vector<xbot_msgs::ActionInfo> &actions);

extern void stop();

extern bool setGPS(bool enabled);

AreaRecordingBehavior AreaRecordingBehavior::INSTANCE;

std::string AreaRecordingBehavior::state_name() {
  return "AREA_RECORDING";
}

Behavior *AreaRecordingBehavior::execute() {
  setGPS(true);
  bool error = false;
  ros::Rate inputDelay(ros::Duration().fromSec(0.1));

  while (ros::ok() && !aborted) {
    mower_map::MapArea result;
    xbot_msgs::MapOverlay result_overlay;

    // clear overlay
    map_overlay_pub.publish(result_overlay);

    has_outline = false;

    sub_state = 0;
    while (ros::ok() && !finished_all && !error && !aborted) {
      if (set_docking_position) {
        geometry_msgs::Pose pos;
        if (getDockingPosition(pos)) {
          ROS_INFO_STREAM("new docking pos = " << pos);

          mower_map::SetDockingPointSrv set_docking_point_srv;
          set_docking_point_srv.request.docking_pose = pos;
          auto result = set_docking_point_client.call(set_docking_point_srv);

          has_first_docking_pos = false;
          update_actions();
        }

        set_docking_position = false;
      }

      if (poly_recording_enabled) {
        update_actions();
        geometry_msgs::Polygon poly;
        // record poly
        if (has_outline) {
          sub_state = 1;
        } else {
          sub_state = 2;
        }
        bool success = recordNewPolygon(poly, result_overlay);
        sub_state = 0;
        if (success) {
          if (!has_outline) {
            // first polygon is outline
            has_outline = true;
            result.area = poly;

            std_msgs::ColorRGBA color;
            color.r = 0.0f;
            color.g = 1.0f;
            color.b = 0.0f;
            color.a = 1.0f;

            marker.color = color;
            marker.id = markers.markers.size() + 1;
            marker.action = visualization_msgs::Marker::ADD;
            markers.markers.push_back(marker);
          } else {
            // we already have an outline, add obstacles
            result.obstacles.push_back(poly);

            std_msgs::ColorRGBA color;
            color.r = 1.0f;
            color.g = 0.0f;
            color.b = 0.0f;
            color.a = 1.0f;

            marker.color = color;
            marker.action = visualization_msgs::Marker::ADD;
            marker.id = markers.markers.size() + 1;
            markers.markers.push_back(marker);
          }

        } else {
          error = true;
          ROS_ERROR_STREAM("Error during poly record");
        }
        marker_array_pub.publish(markers);
        update_actions();
      }

      inputDelay.sleep();
    }

    if (!error && has_outline && (is_mowing_area || is_navigation_area)) {
      if (is_mowing_area) {
        ROS_INFO_STREAM("Area recording completed. Adding mowing area.");
      } else if (is_navigation_area) {
        ROS_INFO_STREAM("Area recording completed. Adding navigation area.");
      }
      mower_map::AddMowingAreaSrv srv;
      srv.request.isNavigationArea = !is_mowing_area;
      srv.request.area = result;
      if (add_mowing_area_client.call(srv)) {
        ROS_INFO_STREAM("Area added successfully");
      } else {
        ROS_ERROR_STREAM("error adding area");
      }
    }

    // reset recording error for next area
    error = false;
    // reset finished all in case we want to record a second area
    finished_all = false;

    has_outline = false;
    update_actions();
  }

  return &IdleBehavior::INSTANCE;
}

void AreaRecordingBehavior::enter() {
  has_outline = false;
  is_mowing_area = false;
  is_navigation_area = false;
  manual_mowing = false;

  update_actions();

  has_first_docking_pos = false;
  has_odom = false;
  poly_recording_enabled = false;
  finished_all = false;
  set_docking_position = false;
  markers = visualization_msgs::MarkerArray();
  paused = aborted = false;

  add_mowing_area_client = n->serviceClient<mower_map::AddMowingAreaSrv>("mower_map_service/add_mowing_area");
  set_docking_point_client = n->serviceClient<mower_map::SetDockingPointSrv>("mower_map_service/set_docking_point");

  marker_pub = n->advertise<visualization_msgs::Marker>("area_recorder/progress_visualization", 10);
  map_overlay_pub = n->advertise<xbot_msgs::MapOverlay>("xbot_monitoring/map_overlay", 10);
  marker_array_pub = n->advertise<visualization_msgs::MarkerArray>("area_recorder/progress_visualization_array", 10);

  ROS_INFO_STREAM("Starting recording area");

  ROS_INFO_STREAM("Subscribing to /joy for user input");

  joy_sub = n->subscribe("/joy", 100, &AreaRecordingBehavior::joy_received, this);

  dock_sub = n->subscribe("/record_dock", 100, &AreaRecordingBehavior::record_dock_received, this);
  polygon_sub = n->subscribe("/record_polygon", 100, &AreaRecordingBehavior::record_polygon_received, this);
  mow_area_sub = n->subscribe("/record_mowing", 100, &AreaRecordingBehavior::record_mowing_received, this);
  nav_area_sub = n->subscribe("/record_navigation", 100, &AreaRecordingBehavior::record_navigation_received, this);

  auto_point_collecting_sub =
      n->subscribe("/record_auto_point_collecting", 100, &AreaRecordingBehavior::record_auto_point_collecting, this);
  collect_point_sub = n->subscribe("/record_collect_point", 100, &AreaRecordingBehavior::record_collect_point, this);

  pose_sub = n->subscribe("/xbot_positioning/xb_pose", 100, &AreaRecordingBehavior::pose_received, this);
}

void AreaRecordingBehavior::exit() {
  for (auto &a : actions) {
    a.enabled = false;
  }
  registerActions("mower_logic:area_recording", actions);

  map_overlay_pub.shutdown();
  marker_pub.shutdown();
  marker_array_pub.shutdown();
  joy_sub.shutdown();
  dock_sub.shutdown();
  polygon_sub.shutdown();
  mow_area_sub.shutdown();
  nav_area_sub.shutdown();
  auto_point_collecting_sub.shutdown();
  collect_point_sub.shutdown();
  pose_sub.shutdown();
  add_mowing_area_client.shutdown();
  set_docking_point_client.shutdown();
}

void AreaRecordingBehavior::reset() {
}

bool AreaRecordingBehavior::needs_gps() {
  return false;
}

bool AreaRecordingBehavior::mower_enabled() {
  return manual_mowing;
}

void AreaRecordingBehavior::pose_received(const xbot_msgs::AbsolutePose::ConstPtr &msg) {
  last_pose = *msg;
  has_odom = true;
}

void AreaRecordingBehavior::joy_received(const sensor_msgs::Joy &joy_msg) {
  if (joy_msg.buttons[1] && !last_joy.buttons[1]) {
    // B was pressed. We toggle recording state
    ROS_INFO_STREAM("B PRESSED");
    poly_recording_enabled = !poly_recording_enabled;
  }
  // Y + up was pressed, we finish the recording for a navigation area
  if ((joy_msg.buttons[3] && joy_msg.axes[7] > 0.5) && !(last_joy.buttons[3] && last_joy.axes[7] > 0.5)) {
    ROS_INFO_STREAM("Y + UP PRESSED, recording navigation area");
    // stop current poly recording
    poly_recording_enabled = false;

    // set finished
    is_mowing_area = false;
    is_navigation_area = true;
    finished_all = true;
  }
  // Y + down was pressed, we finish the recording for a navigation area
  if ((joy_msg.buttons[3] && joy_msg.axes[7] < -0.5) && !(last_joy.buttons[3] && last_joy.axes[7] < -0.5)) {
    ROS_INFO_STREAM("Y + DOWN PRESSED, recording mowing area");
    // stop current poly recording
    poly_recording_enabled = false;

    // set finished
    is_mowing_area = true;
    is_navigation_area = false;
    finished_all = true;
  }

  // X was pressed, set base position if we are not currently recording
  if (joy_msg.buttons[2] && !last_joy.buttons[2]) {
    ROS_INFO_STREAM("X PRESSED");
    set_docking_position = true;
  }

  // use RB button for manual point collecting
  // enable/disable auto point collecting with LB+RB
  if (joy_msg.buttons[5] && !last_joy.buttons[5]) {
    if (joy_msg.buttons[4] && !last_joy.buttons[4]) {
      ROS_INFO_STREAM("LB+RB PRESSED, toggle auto point collecting");
      auto_point_collecting = !auto_point_collecting;
      ROS_INFO_STREAM("Auto point collecting: " << auto_point_collecting);
    } else {
      ROS_INFO_STREAM("RB PRESSED, collect point");
      collect_point = true;
    }
  }

  last_joy = joy_msg;
}

void AreaRecordingBehavior::record_dock_received(std_msgs::Bool state_msg) {
  if (state_msg.data) {
    ROS_INFO_STREAM("Record dock position");
    set_docking_position = true;
  }
}

void AreaRecordingBehavior::record_polygon_received(std_msgs::Bool state_msg) {
  if (state_msg.data) {
    // We toggle recording state
    ROS_INFO_STREAM("Toggle record polygon");
    poly_recording_enabled = !poly_recording_enabled;
  }
}

void AreaRecordingBehavior::record_navigation_received(std_msgs::Bool state_msg) {
  if (state_msg.data) {
    ROS_INFO_STREAM("Save polygon as navigation area");
    // stop current poly recording
    poly_recording_enabled = false;

    // set finished
    is_mowing_area = false;
    is_navigation_area = true;
    finished_all = true;
  }
}

void AreaRecordingBehavior::record_mowing_received(std_msgs::Bool state_msg) {
  if (state_msg.data) {
    ROS_INFO_STREAM("Save polygon as mowing area");
    // stop current poly recording
    poly_recording_enabled = false;

    // set finished
    is_mowing_area = true;
    is_navigation_area = false;
    finished_all = true;
  }
}

bool AreaRecordingBehavior::recordNewPolygon(geometry_msgs::Polygon &polygon, xbot_msgs::MapOverlay &resultOverlay) {
  ROS_INFO_STREAM("recordNewPolygon");

  bool success = true;
  marker = visualization_msgs::Marker();
  marker.header.frame_id = "map";
  marker.ns = "area_recorder";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = 0;
  marker.pose.orientation.w = 1.0f;
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;
  marker.frame_locked = true;

  std_msgs::ColorRGBA color;
  color.b = 1.0f;
  color.a = 1.0f;

  marker.color = color;

  ros::Rate updateRate(10);

  has_odom = false;

  // push a new poly to the visualization overlay
  {
    xbot_msgs::MapOverlayPolygon poly_viz;
    poly_viz.closed = false;
    poly_viz.line_width = 0.1;
    poly_viz.color = "blue";
    resultOverlay.polygons.push_back(poly_viz);
  }
  auto &poly_viz = resultOverlay.polygons.back();

  while (true) {
    if (!ros::ok() || aborted) {
      ROS_WARN_STREAM("Preempting Area Recorder");
      success = false;
      break;
    }

    updateRate.sleep();

    if (!has_odom) continue;

    auto pose_in_map = last_pose.pose.pose;
    if (polygon.points.empty()) {
      // add the first point
      geometry_msgs::Point32 pt;
      pt.x = pose_in_map.position.x;
      pt.y = pose_in_map.position.y;
      pt.z = 0.0;
      //                ROS_INFO_STREAM("Adding First Point: " << pt);

      polygon.points.push_back(pt);
      {
        geometry_msgs::Point vpt;
        vpt.x = pt.x;
        vpt.y = pt.y;
        marker.points.push_back(vpt);
      }

      marker.header.seq++;
      marker.header.stamp = ros::Time::now();
      marker.header.frame_id = "map";

      marker_pub.publish(marker);

      polygon.points.push_back(pt);
      poly_viz.polygon.points.push_back(pt);
      map_overlay_pub.publish(resultOverlay);
    } else {
      auto last = polygon.points.back();
      tf2::Vector3 last_point(last.x, last.y, 0.0);
      tf2::Vector3 current_point(pose_in_map.position.x, pose_in_map.position.y, 0.0);

      bool is_new_point_far_enough = (current_point - last_point).length() > NEW_POINT_MIN_DISTANCE;
      bool is_point_auto_collected = auto_point_collecting && is_new_point_far_enough;
      bool is_point_manual_collected = !auto_point_collecting && collect_point && is_new_point_far_enough;

      if (is_point_auto_collected || is_point_manual_collected) {
        geometry_msgs::Point32 pt;
        pt.x = pose_in_map.position.x;
        pt.y = pose_in_map.position.y;
        pt.z = 0.0;
        //                    ROS_INFO_STREAM("Adding Point: " << pt);
        polygon.points.push_back(pt);
        {
          geometry_msgs::Point vpt;
          vpt.x = pt.x;
          vpt.y = pt.y;
          marker.points.push_back(vpt);
        }

        marker.header.seq++;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = "map";

        marker_pub.publish(marker);

        poly_viz.polygon.points.push_back(pt);
        map_overlay_pub.publish(resultOverlay);

        if (is_point_manual_collected) {
          collect_point = false;
        }
      }
    }

    if (!poly_recording_enabled) {
      if (polygon.points.size() > 2) {
        // add first point to close the poly
        polygon.points.push_back(polygon.points.front());
      } else {
        success = false;
      }
      ROS_INFO_STREAM("Finished Recording polygon");
      break;
    }
  }

  marker.action = visualization_msgs::Marker::DELETE;
  marker_pub.publish(marker);

  // close poly
  poly_viz.closed = true;
  poly_viz.line_width = 0.05;
  if (resultOverlay.polygons.size() == 1) {
    poly_viz.color = "green";
  } else {
    poly_viz.color = "red";
  }
  map_overlay_pub.publish(resultOverlay);

  return success;
}

bool AreaRecordingBehavior::getDockingPosition(geometry_msgs::Pose &pos) {
  if (!has_first_docking_pos) {
    ROS_INFO_STREAM("Recording first docking position");

    auto odom_ptr =
        ros::topic::waitForMessage<xbot_msgs::AbsolutePose>("/xbot_positioning/xb_pose", ros::Duration(1, 0));

    first_docking_pos = odom_ptr->pose.pose;
    has_first_docking_pos = true;
    update_actions();
    return false;
  } else {
    ROS_INFO_STREAM("Recording second docking position");

    auto odom_ptr =
        ros::topic::waitForMessage<xbot_msgs::AbsolutePose>("/xbot_positioning/xb_pose", ros::Duration(1, 0));

    pos.position = odom_ptr->pose.pose.position;

    double yaw = atan2(pos.position.y - first_docking_pos.position.y, pos.position.x - first_docking_pos.position.x);
    tf2::Quaternion docking_orientation(0.0, 0.0, yaw);
    pos.orientation = tf2::toMsg(docking_orientation);

    update_actions();
    return true;
  }
}

void AreaRecordingBehavior::command_home() {
  abort();
}

void AreaRecordingBehavior::command_start() {
}

void AreaRecordingBehavior::command_s1() {
}

void AreaRecordingBehavior::command_s2() {
}

bool AreaRecordingBehavior::redirect_joystick() {
  return true;
}

uint8_t AreaRecordingBehavior::get_sub_state() {
  return sub_state;
}

uint8_t AreaRecordingBehavior::get_state() {
  return mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_RECORDING;
}

std::string AreaRecordingBehavior::sub_state_name() {
  // yes, this doesnt have a sub_sate, but we'll switch to behavior trees anyways. adding a substate here will break
  // stuff
  if (has_first_docking_pos) {
    return "RECORD_DOCKING_POSITION";
  }
  switch (sub_state) {
    case 0: return "";
    case 1: return "RECORD_OUTLINE";
    case 2: return "RECORD_OBSTACLE";
    default: return "";
  }
}

void AreaRecordingBehavior::handle_action(std::string action) {
  if (action == "mower_logic:area_recording/start_recording") {
    ROS_INFO_STREAM("Got start recording");
    poly_recording_enabled = true;
  } else if (action == "mower_logic:area_recording/stop_recording") {
    ROS_INFO_STREAM("Got stop recording");
    poly_recording_enabled = false;
  } else if (action == "mower_logic:area_recording/finish_navigation_area") {
    ROS_INFO_STREAM("Got save navigation area");
    // stop current poly recording
    poly_recording_enabled = false;

    // set finished
    is_mowing_area = false;
    is_navigation_area = true;
    finished_all = true;
  } else if (action == "mower_logic:area_recording/finish_mowing_area") {
    ROS_INFO_STREAM("Got save mowing area");
    // stop current poly recording
    poly_recording_enabled = false;

    // set finished
    is_mowing_area = true;
    is_navigation_area = false;
    finished_all = true;
  } else if (action == "mower_logic:area_recording/finish_discard") {
    ROS_INFO_STREAM("Got discard recorded area");
    // stop current poly recording
    poly_recording_enabled = false;

    // set finished
    is_mowing_area = false;
    is_navigation_area = false;
    finished_all = true;
  } else if (action == "mower_logic:area_recording/exit_recording_mode") {
    ROS_INFO_STREAM("Got exit without saving");
    // stop current poly recording
    poly_recording_enabled = false;

    // set finished
    is_mowing_area = false;
    is_navigation_area = false;
    finished_all = true;
    abort();
  } else if (action == "mower_logic:area_recording/record_dock") {
    ROS_INFO_STREAM("Got record dock");
    set_docking_position = true;
  } else if (action == "mower_logic:area_recording/auto_point_collecting_enable") {
    ROS_INFO_STREAM("Got enable auto point collecting");
    auto_point_collecting = true;
  } else if (action == "mower_logic:area_recording/auto_point_collecting_disable") {
    ROS_INFO_STREAM("Got disable auto point collecting");
    auto_point_collecting = false;
  } else if (action == "mower_logic:area_recording/collect_point") {
    ROS_INFO_STREAM("Got collect point");
    collect_point = true;
  } else if (action == "mower_logic:area_recording/start_manual_mowing") {
    ROS_INFO_STREAM("Starting manual mowing");
    manual_mowing = true;
  } else if (action == "mower_logic:area_recording/stop_manual_mowing") {
    ROS_INFO_STREAM("Stopping manual mowing");
    manual_mowing = false;
  }
  update_actions();
}

AreaRecordingBehavior::AreaRecordingBehavior() {
  xbot_msgs::ActionInfo start_recording_action;
  start_recording_action.action_id = "start_recording";
  start_recording_action.enabled = false;
  start_recording_action.action_name = "Start Recording";

  xbot_msgs::ActionInfo stop_recording_action;
  stop_recording_action.action_id = "stop_recording";
  stop_recording_action.enabled = false;
  stop_recording_action.action_name = "Stop Recording";

  xbot_msgs::ActionInfo finish_navigation_area_action;
  finish_navigation_area_action.action_id = "finish_navigation_area";
  finish_navigation_area_action.enabled = false;
  finish_navigation_area_action.action_name = "Save Navigation Area";

  xbot_msgs::ActionInfo finish_mowing_area_action;
  finish_mowing_area_action.action_id = "finish_mowing_area";
  finish_mowing_area_action.enabled = false;
  finish_mowing_area_action.action_name = "Save Mowing Area";

  xbot_msgs::ActionInfo exit_recording_mode_action;
  exit_recording_mode_action.action_id = "exit_recording_mode";
  exit_recording_mode_action.enabled = false;
  exit_recording_mode_action.action_name = "Exit";

  xbot_msgs::ActionInfo finish_discard_action;
  finish_discard_action.action_id = "finish_discard";
  finish_discard_action.enabled = false;
  finish_discard_action.action_name = "Discard Area";

  xbot_msgs::ActionInfo record_dock_action;
  record_dock_action.action_id = "record_dock";
  record_dock_action.enabled = false;
  record_dock_action.action_name = "Record Docking point";

  xbot_msgs::ActionInfo auto_point_collecting_enable_action;
  auto_point_collecting_enable_action.action_id = "auto_point_collecting_enable";
  auto_point_collecting_enable_action.enabled = false;
  auto_point_collecting_enable_action.action_name = "Enable automatic point collecting";

  xbot_msgs::ActionInfo auto_point_collecting_disable_action;
  auto_point_collecting_disable_action.action_id = "auto_point_collecting_disable";
  auto_point_collecting_disable_action.enabled = false;
  auto_point_collecting_disable_action.action_name = "Disable automatic point collecting";

  xbot_msgs::ActionInfo collect_point_action;
  collect_point_action.action_id = "collect_point";
  collect_point_action.enabled = false;
  collect_point_action.action_name = "Collect point";

  xbot_msgs::ActionInfo start_manual_mowing_action;
  start_manual_mowing_action.action_id = "start_manual_mowing";
  start_manual_mowing_action.enabled = false;
  start_manual_mowing_action.action_name = "Start manual mowing";

  xbot_msgs::ActionInfo stop_manual_mowing_action;
  stop_manual_mowing_action.action_id = "stop_manual_mowing";
  stop_manual_mowing_action.enabled = false;
  stop_manual_mowing_action.action_name = "Stop manual mowing";

  actions.clear();
  actions.push_back(start_recording_action);
  actions.push_back(stop_recording_action);
  actions.push_back(finish_navigation_area_action);
  actions.push_back(finish_mowing_area_action);
  actions.push_back(exit_recording_mode_action);
  actions.push_back(finish_discard_action);
  actions.push_back(record_dock_action);
  actions.push_back(auto_point_collecting_enable_action);
  actions.push_back(auto_point_collecting_disable_action);
  actions.push_back(collect_point_action);
  actions.push_back(start_manual_mowing_action);
  actions.push_back(stop_manual_mowing_action);
}

void AreaRecordingBehavior::update_actions() {
  {
    for (auto &a : actions) {
      a.enabled = false;
    }
    if (has_first_docking_pos) {
      // we have recorded the first docking pose, only option is to finish by recording second one
      actions[6].enabled = true;
    } else if (poly_recording_enabled) {
      // currently recording a polygon, allow stop and save actions
      actions[1].enabled = true;
      actions[2].enabled = true;
      actions[3].enabled = true;
      actions[4].enabled = true;
      actions[5].enabled = true;

      // enable/disable auto point collecting
      actions[7].enabled = !auto_point_collecting;
      actions[8].enabled = auto_point_collecting;
      actions[9].enabled = !auto_point_collecting;
    } else {
      // neither recording a polygon nor docking point. we can save if we have an outline and always discard
      if (has_outline) {
        actions[0].enabled = true;
        actions[2].enabled = true;
        actions[3].enabled = true;
        actions[4].enabled = true;
        actions[5].enabled = true;
      } else {
        // enable start recording, discard area and record dock
        actions[0].enabled = true;
        actions[4].enabled = true;
        actions[6].enabled = true;
      }
    }
    // start_manual_mowing
    actions[10].enabled = !manual_mowing;
    // stop manual mowing
    actions[11].enabled = manual_mowing;

    registerActions("mower_logic:area_recording", actions);
  }
}

void AreaRecordingBehavior::record_auto_point_collecting(std_msgs::Bool state_msg) {
  if (state_msg.data) {
    ROS_INFO_STREAM("Recording auto point collecting enabled");
    auto_point_collecting = true;
  } else {
    ROS_INFO_STREAM("Recording auto point collecting disabled");
    auto_point_collecting = false;
  }
}

void AreaRecordingBehavior::record_collect_point(std_msgs::Bool state_msg) {
  if (state_msg.data) {
    ROS_INFO_STREAM("Recording collect point");
    collect_point = true;
  }
}
