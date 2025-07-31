#ifndef GPS_SERVICE_HPP
#define GPS_SERVICE_HPP

#include <GpsServiceBase.hpp>

using namespace xbot::service;

class GpsService : public GpsServiceBase {
 public:
  explicit GpsService(uint16_t service_id) : GpsServiceBase(service_id) {
  }

 private:
  void tick();
  ServiceSchedule tick_schedule_{*this, 200'000, XBOT_FUNCTION_FOR_METHOD(GpsService, &GpsService::tick, this)};
};

#endif  // GPS_SERVICE_HPP
