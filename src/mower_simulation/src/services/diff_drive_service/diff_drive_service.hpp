//
// Created by clemens on 26.07.24.
//

#ifndef DIFF_DRIVE_SERVICE_HPP
#define DIFF_DRIVE_SERVICE_HPP

#include <DiffDriveServiceBase.hpp>
#include "../../SimRobot.h"

class DiffDriveService : public DiffDriveServiceBase {

 public:
  explicit DiffDriveService(uint16_t service_id, SimRobot &robot)
      : DiffDriveServiceBase(service_id, 20000), robot_(robot) {
  }

  void OnMowerStatusChanged(uint32_t new_status);

 protected:
  bool Configure() override;
  void OnStart() override;
  void OnCreate() override;
  void OnStop() override;

 private:
  SimRobot &robot_;
  void tick() override;

  void SetDuty();
  void ProcessStatusUpdate();

 protected:
  bool OnControlTwistChanged(const double* new_value, uint32_t length) override;
};

#endif  // DIFF_DRIVE_SERVICE_HPP
