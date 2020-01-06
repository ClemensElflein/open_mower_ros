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

#include "vesc_hi/vesc_servo_controller.h"

VescServoController::VescServoController(ros::NodeHandle nh, VescInterface* interface_ptr) {
    // initializes members
    if(interface_ptr == NULL) {
        ros::shutdown();
    } else {
        interface_ptr_ = interface_ptr;
    }

    calibration_flag_ = true;
    error_previous_   = 0;

    // reads parameters
    nh.param("servo/k_p", Kp_, 50.0);
    nh.param("servo/k_i", Ki_, 0.0);
    nh.param("servo/k_d", Kd_, 1.0);
    nh.param("servo/calibration_current", calibration_current_, 6.0);
}

void VescServoController::executeCalibration() {
    calibration_flag_ = true;

    return;
}

bool VescServoController::calibrate(const double position_current) {
    static double   position_previous;
    static uint16_t step;

    // sets current for calibration
    interface_ptr_->setCurrent(6.0);

    if(step % 10 == 0 && position_current == position_previous) {
        // calibration finishes
        calibration_flag_ = false;
        step              = 0;

        return true;
    } else {
        // continues calibration
        position_previous = position_current;
        step++;

        return false;
    }
}
