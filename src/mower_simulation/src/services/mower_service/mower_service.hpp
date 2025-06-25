#ifndef MOWER_SERVICE_HPP
#define MOWER_SERVICE_HPP

#include <MowerServiceBase.hpp>

using namespace xbot::service;

class MowerService : public MowerServiceBase {
 public:
  explicit MowerService(const uint16_t service_id) : MowerServiceBase(service_id) {
  }

 protected:
  bool OnStart() override;
  void OnMowerEnabledChanged(const uint8_t &new_value) override;

 private:
  bool mower_running_ = false;
  void tick();
  ServiceSchedule tick_schedule_{*this, 1'000'000, XBOT_FUNCTION_FOR_METHOD(MowerService, &MowerService::tick, this)};
};

#endif  // MOWER_SERVICE_HPP
