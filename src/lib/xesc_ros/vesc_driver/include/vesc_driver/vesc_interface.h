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

#ifndef VESC_DRIVER_VESC_INTERFACE_H_
#define VESC_DRIVER_VESC_INTERFACE_H_

#include <exception>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <chrono>
#include <condition_variable>

#include <pthread.h>
#include <mutex>
#include <serial/serial.h>
#include <boost/crc.hpp>
#include <boost/noncopyable.hpp>

#include "vesc_driver/vesc_packet.h"
#include "vesc_driver/vesc_packet_factory.h"

namespace vesc_driver {
    enum VESC_CONNECTION_STATE {
        DISCONNECTED = 0,
        WAITING_FOR_FW = 1,
        CONNECTED_INCOMPATIBLE_FW = 2,
        CONNECTED = 3,
    };

    struct VescStatusStruct {
        uint32_t seq;
        uint8_t fw_version_major;
        uint8_t fw_version_minor;
        VESC_CONNECTION_STATE connection_state;
        double voltage_input;        // input voltage (volt)
        double temperature_pcb;      // temperature of printed circuit board (degrees Celsius)
        double temperature_motor;      // temperature of printed circuit board (degrees Celsius)
        double current_motor;        // motor current (ampere)
        double current_input;        // input current (ampere)
        double speed_erpm;                // motor velocity (rad/s)
        double duty_cycle;           // duty cycle (0 to 1)
        double charge_drawn;         // electric charge drawn from input (ampere-hour)
        double charge_regen;         // electric charge regenerated to input (ampere-hour)
        double energy_drawn;         // energy drawn from input (watt-hour)
        double energy_regen;         // energy regenerated to input (watt-hour)
        double displacement;         // net tachometer (counts)
        double distance_traveled;    // total tachnometer (counts)
        uint32_t tacho;
        uint32_t tacho_absolute;
        bool direction;
        int32_t fault_code;
    };

/**
 * Class providing an interface to the Vedder VESC motor controller via a serial port interface.
 */
    class VescInterface : private boost::noncopyable {
    public:
        typedef std::function<void(const std::string &)> ErrorHandlerFunction;

        /**
         * Creates a VescInterface object. Opens the serial port interface to the VESC if @p port is not
         * empty, otherwise the serial port remains closed until connect() is called.
         *
         * @param port Address of the serial port, e.g. '/dev/ttyUSB0'.
         *
         */
        VescInterface(const ErrorHandlerFunction &error_handler, uint32_t state_request_millis = 20);

        /**
         * VescInterface destructor.
         */
        ~VescInterface();

        void setDutyCycle(double duty_cycle);

        void setCurrent(double current);

        void setBrake(double brake);

        void setSpeed(double speed);

        void setPosition(double position);

        void start(const std::string &port);

        void stop();

        void get_status(VescStatusStruct *status);
        void wait_for_status(VescStatusStruct *status);
        void requestFWVersion();
        void requestState();

    private:
        /**
         * Send a VESC packet.
         */
        bool send(const VescPacket &packet);

        static void *rx_thread_helper(void *context) {
            return ((VescInterface *) context)->rx_thread();
        }
        static void *update_thread_helper(void *context) {
            return ((VescInterface *) context)->update_thread();
        }


        void *rx_thread();
        void *update_thread();

        void handle_packet(VescPacketConstPtr packet);


        pthread_t rx_thread_handle_;
        pthread_t update_thread_handle_;
        bool rx_thread_run_;
        bool update_thread_run_;


        ErrorHandlerFunction error_handler_;
        serial::Serial serial_;
        std::string port_;
        std::mutex status_mutex_;
        // since multiple threads will call the send() function, we need a mutex.
        std::mutex serial_tx_mutex_;
        std::condition_variable status_cv_;
        struct VescStatusStruct status_;

        uint32_t state_request_millis;
    };

}  // namespace xesc_2040_driver

#endif  // VESC_DRIVER_VESC_INTERFACE_H_
