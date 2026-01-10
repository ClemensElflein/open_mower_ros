/*
 * OpenMower V2 Firmware
 * Part of open_mower_ros (https://github.com/ClemensElflein/open_mower_ros)
 *
 * Copyright (C) 2026 The OpenMower Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/**
 * @file HighLevelServiceInterface.h
 * @brief Service interface for high-level mower status communication
 * @author Apehaenger <joerg@ebeling.ws>
 * @date 2026-01-10
 */

#ifndef HIGHLEVELSERVICEINTERFACE_H
#define HIGHLEVELSERVICEINTERFACE_H

#include <mower_msgs/HighLevelStatus.h>
#include <ros/ros.h>

#include <HighLevelServiceInterfaceBase.hpp>

class HighLevelServiceInterface : public HighLevelServiceInterfaceBase {
 public:
  HighLevelServiceInterface(uint16_t service_id, const xbot::serviceif::Context& ctx)
      : HighLevelServiceInterfaceBase(service_id, ctx) {
    ros::NodeHandle nh;
    high_level_status_sub_ =
        nh.subscribe("mower_logic/current_state", 1, &HighLevelServiceInterface::highLevelStatusReceived, this);
  }

 protected:
  // Called when the firmware sends an output (Action string)
  void OnActionChanged(const char* new_value, uint32_t length) override;

 private:
  void highLevelStatusReceived(const mower_msgs::HighLevelStatus::ConstPtr& msg);
  ros::Subscriber high_level_status_sub_;
};

#endif  // HIGHLEVELSERVICEINTERFACE_H