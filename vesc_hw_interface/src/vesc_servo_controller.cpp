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

#include "vesc_hw_interface/vesc_servo_controller.h"

namespace vesc_hw_interface {

void VescServoController::init(ros::NodeHandle nh, VescInterface* interface_ptr, const double frequency) {
    // initializes members
    if(interface_ptr == NULL) {
        ros::shutdown();
    } else {
        interface_ptr_ = interface_ptr;
    }

    calibration_flag_ = true;
    zero_position_    = 0.0;
    frequency_        = frequency;

    // reads parameters
    nh.param("vesc_hw_interface/servo/Kp", Kp_, 50.0);
    nh.param("vesc_hw_interface/servo/Ki", Ki_, 0.0);
    nh.param("vesc_hw_interface/servo/Kd", Kd_, 1.0);
    nh.param("vesc_hw_interface/servo/calibration_current", calibration_current_, 6.0);
    nh.param("vesc_hw_interface/servo/calibration_position", calibration_position_, 0.0);

    return;
}

void VescServoController::control(const double position_reference, const double position_current) {
    // executes caribration
    if(calibration_flag_) {
        calibrate(position_current);

        return;
    }

    static double error_previous, error_integ;

    // calculates PD control
    double error_current = position_reference - position_current;
    double u_pd          = Kp_ * error_current + Kd_ * (error_current - error_previous) * frequency_;

    double u, u_pid;

    // calculates I control if PD input is not saturated
    if(isSaturated(u_pd)) {
        u = saturate(u_pd);
    } else {
        double error_integ_new = error_integ + (error_current + error_previous) / 2.0 / frequency_;
        u_pid                  = u_pd + Ki_ * error_integ_new;

        // not use I control if PID input is saturated
        // since error integration causes bugs
        if(isSaturated(u_pid)) {
            u = u_pd;
        } else {
            u           = u_pid;
            error_integ = error_integ_new;
        }
    }

    // saves current error
    error_previous = error_current;

    // command duty
    interface_ptr_->setDutyCycle(u);

    return;
}

double VescServoController::getZeroPosition() const {
    return zero_position_;
}

void VescServoController::executeCalibration() {
    calibration_flag_ = true;

    return;
}

bool VescServoController::calibrate(const double position_current) {
    static double   position_previous;
    static uint16_t step;

    // sets current for calibration
    interface_ptr_->setCurrent(calibration_current_);
    step++;

    if(step % 20 == 0 && position_current == position_previous) {
        // finishes calibrating
        interface_ptr_->setCurrent(0.0);
        calibration_flag_ = false;
        step              = 0;
        zero_position_    = position_current - calibration_position_;

        ROS_INFO("Calibration Finished");
        return true;
    } else {
        // continues calibration
        position_previous = position_current;

        return false;
    }
}

bool VescServoController::isSaturated(const double arg) const {
    if(std::abs(arg) > 1.0) {
        return true;
    } else {
        return false;
    }
}

double VescServoController::saturate(const double arg) const {
    if(arg > 1.0) {
        return 1.0;
    } else if(arg < -1.0) {
        return -1.0;
    } else {
        return arg;
    }
}

}  // namespace vesc_hw_interface
