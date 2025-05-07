//
// Created by clemens on 31.07.24.
//

#include "imu_service.hpp"

void ImuService::tick() {
  double axes[9]{};

  double unused;
  robot_.GetTwist(unused, axes[5]);

  SendAxes(axes, 9);
}
