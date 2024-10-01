/*********************************************************************
 * Copyright (c) 2019, SoftBank Corp.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Softbank Corp. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************/

/** NOTE *************************************************************
 * This program had been developed by Michael T. Boulet at MIT under
 * the BSD 3-clause License until Dec. 2016. Since Nov. 2019, Softbank
 * Corp. takes over development as new packages.
 ********************************************************************/

#include "vesc_driver/vesc_driver.h"

namespace vesc_driver {
    VescDriver::VescDriver(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
            : vesc_(std::bind(&VescDriver::vescErrorCallback, this, std::placeholders::_1)),
              duty_cycle_limit_(private_nh, "duty_cycle", -1.0, 1.0) {
        // get vesc serial port address
        std::string port;
        if (!private_nh.getParam("serial_port", port)) {
            ROS_FATAL("VESC communication port parameter required.");
            throw ros::InvalidParameterException("VESC communication port parameter required.");
        }

        // get motor pole pairs, just needed for eRPM to RPM calculation
        if (!private_nh.getParam("motor_pole_pairs", pole_pairs)) {
            ROS_WARN("VESC config misses motor_pole_pairs parameter. Assuming 4.");
            pole_pairs = 4;
        }
        if(pole_pairs == 0) {
            ROS_WARN("VESC config has wrong motor_pole_pairs value %i. Forced to 1.", pole_pairs);
            pole_pairs = 1;
        }

        vesc_.start(port);
    }


    void VescDriver::vescErrorCallback(const std::string &error) {
        ROS_ERROR("%s", error.c_str());
    }


    void VescDriver::stop() {
        vesc_.stop();
    }

    void VescDriver::getStatus(xesc_msgs::XescStateStamped &state_msg) {
        vesc_.get_status(&vesc_status);

        state_msg.header.stamp = ros::Time::now();
        state_msg.state.connection_state = vesc_status.connection_state;
        state_msg.state.fw_major = vesc_status.fw_version_major;
        state_msg.state.fw_minor = vesc_status.fw_version_minor;
        state_msg.state.voltage_input = vesc_status.voltage_input;
        state_msg.state.temperature_pcb = vesc_status.temperature_pcb;
        state_msg.state.temperature_motor = vesc_status.temperature_motor;
        state_msg.state.current_input = vesc_status.current_input;
        state_msg.state.duty_cycle = vesc_status.duty_cycle;
        state_msg.state.tacho = vesc_status.tacho;
        state_msg.state.fault_code = vesc_status.fault_code;
        state_msg.state.tacho_absolute = vesc_status.tacho_absolute;
        state_msg.state.direction = vesc_status.direction;
        state_msg.state.rpm = vesc_status.speed_erpm / pole_pairs;
    }

    void VescDriver::getStatusBlocking(xesc_msgs::XescStateStamped &state_msg) {
        vesc_.wait_for_status(&vesc_status);

        state_msg.header.stamp = ros::Time::now();
        state_msg.state.connection_state = vesc_status.connection_state;
        state_msg.state.fw_major = vesc_status.fw_version_major;
        state_msg.state.fw_minor = vesc_status.fw_version_minor;
        state_msg.state.voltage_input = vesc_status.voltage_input;
        state_msg.state.temperature_pcb = vesc_status.temperature_pcb;
        state_msg.state.temperature_motor = vesc_status.temperature_motor;
        state_msg.state.current_input = vesc_status.current_input;
        state_msg.state.duty_cycle = vesc_status.duty_cycle;
        state_msg.state.tacho = vesc_status.tacho;
        state_msg.state.fault_code = vesc_status.fault_code;
        state_msg.state.tacho_absolute = vesc_status.tacho_absolute;
        state_msg.state.direction = vesc_status.direction;
        state_msg.state.rpm = vesc_status.speed_erpm / pole_pairs;
    }

    void VescDriver::setDutyCycle(float duty_cycle) {
        vesc_.setDutyCycle(duty_cycle);
    }


    VescDriver::CommandLimit::CommandLimit(const ros::NodeHandle &nh, const std::string &str,
                                           const boost::optional<double> &min_lower,
                                           const boost::optional<double> &max_upper)
            : name(str) {
        // check if user's minimum value is outside of the range min_lower to max_upper
        double param_min;
        if (nh.getParam(name + "_min", param_min)) {
            if (min_lower && param_min < *min_lower) {
                lower = *min_lower;
                ROS_WARN_STREAM("Parameter " << name << "_min (" << param_min << ") is less than the feasible minimum ("
                                             << *min_lower << ").");
            } else if (max_upper && param_min > *max_upper) {
                lower = *max_upper;
                ROS_WARN_STREAM(
                        "Parameter " << name << "_min (" << param_min << ") is greater than the feasible maximum ("
                                     << *max_upper << ").");
            } else {
                lower = param_min;
            }
        } else if (min_lower) {
            lower = *min_lower;
        }

        // check if the uers' maximum value is outside of the range min_lower to max_upper
        double param_max;
        if (nh.getParam(name + "_max", param_max)) {
            if (min_lower && param_max < *min_lower) {
                upper = *min_lower;
                ROS_WARN_STREAM("Parameter " << name << "_max (" << param_max << ") is less than the feasible minimum ("
                                             << *min_lower << ").");
            } else if (max_upper && param_max > *max_upper) {
                upper = *max_upper;
                ROS_WARN_STREAM(
                        "Parameter " << name << "_max (" << param_max << ") is greater than the feasible maximum ("
                                     << *max_upper << ").");
            } else {
                upper = param_max;
            }
        } else if (max_upper) {
            upper = *max_upper;
        }

        // check for min > max
        if (upper && lower && *lower > *upper) {
            ROS_WARN_STREAM(
                    "Parameter " << name << "_max (" << *upper << ") is less than parameter " << name << "_min ("
                                 << *lower << ").");
            double temp(*lower);
            lower = *upper;
            upper = temp;
        }

        std::ostringstream oss;
        oss << "  " << name << " limit: ";
        if (lower)
            oss << *lower << " ";
        else
            oss << "(none) ";
        if (upper)
            oss << *upper;
        else
            oss << "(none)";
        ROS_DEBUG_STREAM(oss.str());
    }

    double VescDriver::CommandLimit::clip(double value) {
        if (lower && value < lower) {
            ROS_INFO_THROTTLE(10, "%s command value (%f) below minimum limit (%f), clipping.", name.c_str(), value,
                              *lower);
            return *lower;
        }
        if (upper && value > upper) {
            ROS_INFO_THROTTLE(10, "%s command value (%f) above maximum limit (%f), clipping.", name.c_str(), value,
                              *upper);
            return *upper;
        }
        return value;
    }

}  // namespace xesc_2040_driver
