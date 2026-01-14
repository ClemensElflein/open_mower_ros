/*
 * OpenMower V2 Firmware
 * Part of open_mower_ros (https://github.com/ClemensElflein/open_mower_ros)
 *
 * Copyright (C) 2026 The OpenMower Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/**
 * @file HighLevelServiceInterface.cpp
 * @brief Service interface implementation for high-level mower status communication
 * @author Apehaenger <joerg@ebeling.ws>
 * @date 2026-01-10
 */

#include "HighLevelServiceInterface.h"

#include <ros/console.h>

void HighLevelServiceInterface::highLevelStatusReceived(const mower_msgs::HighLevelStatus::ConstPtr& msg) {
  // Direct cast from ROS state to enum HighLevelStatus
  HighLevelStatus state_id = static_cast<HighLevelStatus>(msg->state);
  SendStateID(state_id);

  // State Name (string)
  SendStateName(msg->state_name.c_str(), msg->state_name.size());

  // Sub State Name (string)
  SendSubStateName(msg->sub_state_name.c_str(), msg->sub_state_name.size());

  // GPS Quality (float)
  SendGpsQuality(msg->gps_quality_percent);

  // Current Area (int16_t)
  SendCurrentArea(msg->current_area);

  // Current Path (int16_t)
  SendCurrentPath(msg->current_path);

  // Current Path Index (int16_t)
  SendCurrentPathIndex(msg->current_path_index);
}

void HighLevelServiceInterface::OnActionChanged(const char* new_value, uint32_t length) {
  // The firmware sent an action string. For now, just log it.
  ROS_INFO_STREAM("HighLevelService received action: " << std::string(new_value, length));
  // TODO: Implement action handling (e.g., publish to xbot/action topic)
}
