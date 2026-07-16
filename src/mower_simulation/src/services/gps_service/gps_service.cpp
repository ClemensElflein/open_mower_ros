//
// Created by clemens on 02.08.24.
//

#include "gps_service.hpp"

void GpsService::tick() {
  StartTransaction();
  // Good GPS = RTK fix (~2 cm); bad GPS = no RTK fix / RTK float (~1 m). Driven by the
  // sim.gps.set RPC via SimRobot.
  const bool gps_good = robot_.IsGpsGood();
  std::string fix_type = gps_good ? "FIX" : "FLOAT";
  SendFixType(fix_type.c_str(), fix_type.length());
  SendPositionHorizontalAccuracy(gps_good ? 0.02 : 1.0);
  double xyz[3]{};
  double headingAndAccuracy[2];
  headingAndAccuracy[1] = M_PI / 10.0;
  double vx, vr;
  robot_.GetTwist(vx, vr);
  double center_x, center_y;
  robot_.GetPosition(center_x, center_y, headingAndAccuracy[0]);
  const double heading = headingAndAccuracy[0];
  // xbot_positioning's PositionMeasurementModel expects the GPS antenna's position, i.e.
  // the robot center plus the (heading-rotated) antenna offset - not the center itself.
  // Sending the bare center here would make the EKF perceive a constant, heading-
  // dependent (hence apparently "sideways" as the robot turns) position error.
  xyz[0] = center_x + cos(heading) * antenna_offset_x_ - sin(heading) * antenna_offset_y_;
  xyz[1] = center_y + sin(heading) * antenna_offset_x_ + cos(heading) * antenna_offset_y_;
  SendPosition(xyz, 3);
  // SendMotionHeadingAndAccuracy(headingAndAccuracy, 2);
  // The antenna is a rigid point offset from the center of rotation, so it picks up an
  // extra body-frame velocity component from rotation itself (v = v_center + w x r) -
  // otherwise turning in place would look like the antenna doesn't move at all, when it
  // actually sweeps an arc proportional to the offset.
  const double body_vx = vx - vr * antenna_offset_y_;
  const double body_vy = vr * antenna_offset_x_;
  double motion_vector_enu[3]{};
  motion_vector_enu[0] = body_vx * cos(heading) - body_vy * sin(heading);
  motion_vector_enu[1] = body_vx * sin(heading) + body_vy * cos(heading);
  SendMotionVectorENU(motion_vector_enu, 3);
  CommitTransaction();
}
