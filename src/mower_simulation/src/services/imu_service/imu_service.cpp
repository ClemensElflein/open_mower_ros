//
// Created by clemens on 31.07.24.
//

#include "imu_service.hpp"

bool ImuService::Configure() { return true; }
void ImuService::OnStart() {}
void ImuService::OnStop() {}
void ImuService::OnCreate() {

}
void ImuService::tick() {
  double axes[9]{};

  double unused;
  robot_.GetTwist(unused, axes[5]);

  SendAxes(axes, 9);
}
