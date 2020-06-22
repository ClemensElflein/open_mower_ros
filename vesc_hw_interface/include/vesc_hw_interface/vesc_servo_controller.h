/*********************************************************************
*
* Copyright (c) 2019, SoftBank corp.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
*********************************************************************/

#ifndef VESC_HW_INTERFACE_VESC_SERVO_CONTROLLER_H_
#define VESC_HW_INTERFACE_VESC_SERVO_CONTROLLER_H_

#include <cmath>

#include <ros/ros.h>
#include <vesc_driver/vesc_interface.h>

namespace vesc_hw_interface {

using vesc_driver::VescInterface;

class VescServoController {
public:
    void   init(ros::NodeHandle, VescInterface*, const double);
    void   control(const double, const double);
    double getZeroPosition() const;
    void   executeCalibration();

private:
    VescInterface* interface_ptr_;

    bool   calibration_flag_;
    double calibration_current_, zero_position_;
    double Kp_, Ki_, Kd_, frequency_;

    bool   calibrate(const double);
    bool   isSaturated(const double) const;
    double saturate(const double) const;
};

}  // namespace vesc_hw_interface

#endif  // VESC_HW_INTERFACE_VESC_SERVO_CONTROLLER_H_
