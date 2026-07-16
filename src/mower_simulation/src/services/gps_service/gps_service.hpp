//
// Created by clemens on 02.08.24.
//

#ifndef GPS_SERVICE_HPP
#define GPS_SERVICE_HPP
#include <GpsServiceBase.hpp>

#include "../../SimRobot.h"

using namespace xbot::service;

class GpsService : public GpsServiceBase {
 public:
  explicit GpsService(uint16_t service_id, SimRobot& robot) : GpsServiceBase(service_id), robot_(robot) {
    // xbot_positioning's PositionMeasurementModel expects the incoming GPS position to
    // be the *antenna's* position (robot center + this offset, rotated by heading), not
    // the robot center itself - read the same params it uses so a real GPS fix and our
    // simulated one are directly comparable, and the EKF isn't fed a position that's
    // silently missing the offset it expects to compensate for.
    ros::NodeHandle positioning_nh("/xbot_positioning");
    positioning_nh.param("antenna_offset_x", antenna_offset_x_, 0.0);
    positioning_nh.param("antenna_offset_y", antenna_offset_y_, 0.0);
  }

 private:
  void tick();
  ManagedSchedule tick_schedule_{scheduler_, IsRunning(), 200'000,
                                 XBOT_FUNCTION_FOR_METHOD(GpsService, &GpsService::tick, this)};

 private:
  SimRobot& robot_;
  double antenna_offset_x_ = 0.0;
  double antenna_offset_y_ = 0.0;
};

#endif  // GPS_SERVICE_HPP
