//
// Created by clemens on 02.08.24.
//

#include "position_service.hpp"
bool PositionService::Configure() {
  // We're always happy
  return true;
}
void PositionService::OnStart() {
}
void PositionService::OnCreate() {
}
void PositionService::OnStop() {
}
void PositionService::tick() {
  StartTransaction();
  std::string fix_type = "FIX";
  SendFixType(fix_type.c_str(), fix_type.length());
  SendPositionAccuracy(0.05);
  double xyz[3]{};
  double headingAndAccuracy[2];
  headingAndAccuracy[1] = M_PI / 10.0;
  double vx, vr;
  robot_.GetTwist(vx, vr);
  robot_.GetPosition(xyz[0], xyz[1], headingAndAccuracy[0]);
  SendPositionXYZ(xyz, 3);
  SendMotionHeadingAndAccuracy(headingAndAccuracy, 2);
  double motion_vector_xyz[3]{};
  motion_vector_xyz[0] = vx * cos(headingAndAccuracy[0]);
  motion_vector_xyz[1] = vx * sin(headingAndAccuracy[0]);
  SendMotionVectorXYZ(motion_vector_xyz,3);
  CommitTransaction();
}
bool PositionService::OnRTCMChanged(const uint8_t* new_value, uint32_t length) {
  return true;
}
