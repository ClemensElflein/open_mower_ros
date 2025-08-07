#include "imu_service.hpp"

#include "../../services.hpp"

void ImuService::tick() {
  double axes[9]{};

  double unused;
  robot.GetTwist(unused, axes[5]);

  SendAxes(axes, 9);
}
