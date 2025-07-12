#ifndef DIFF_DRIVE_SERVICE_HPP
#define DIFF_DRIVE_SERVICE_HPP

#include <DiffDriveServiceBase.hpp>

using namespace xbot::service;

class DiffDriveService : public DiffDriveServiceBase {
 public:
  explicit DiffDriveService(uint16_t service_id) : DiffDriveServiceBase(service_id) {
  }

 protected:
  bool OnStart() override;
  void OnStop() override;
  void OnControlTwistChanged(const double *new_value, uint32_t length) override;

 private:
  void tick();
  ServiceSchedule tick_schedule_{*this, 20'000,
                                 XBOT_FUNCTION_FOR_METHOD(DiffDriveService, &DiffDriveService::tick, this)};
};

#endif  // DIFF_DRIVE_SERVICE_HPP
