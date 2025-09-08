#include "gps_service.hpp"

#include "../../services.hpp"

void GpsService::tick() {
  StartTransaction();
  std::string fix_type = "FIX";
  SendFixType(fix_type.c_str(), fix_type.length());
  SendPositionHorizontalAccuracy(0.05);
  double xyz[3]{};
  double headingAndAccuracy[2];
  headingAndAccuracy[1] = M_PI / 10.0;
  double vx, vr;
  robot.GetTwist(vx, vr);
  robot.GetPosition(xyz[0], xyz[1], headingAndAccuracy[0]);
  SendPosition(xyz, 3);
  // SendMotionHeadingAndAccuracy(headingAndAccuracy, 2);
  double motion_vector_enu[3]{};
  motion_vector_enu[0] = vx * cos(headingAndAccuracy[0]);
  motion_vector_enu[1] = vx * sin(headingAndAccuracy[0]);
  SendMotionVectorENU(motion_vector_enu, 3);
  CommitTransaction();
}
