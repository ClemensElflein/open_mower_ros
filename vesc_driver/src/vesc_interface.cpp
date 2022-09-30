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

#include "vesc_driver/vesc_interface.h"

namespace vesc_driver {

    VescInterface::VescInterface(const ErrorHandlerFunction &error_handler, uint32_t state_request_millis)
            : serial_(std::string(), 115200, serial::Timeout::simpleTimeout(100), serial::eightbits,
                      serial::parity_none,
                      serial::stopbits_one, serial::flowcontrol_none),
              state_request_millis(state_request_millis) {

        error_handler_ = error_handler;
    }

    VescInterface::~VescInterface() {
        stop();
    }

    void *VescInterface::update_thread() {
        std::chrono::time_point<std::chrono::steady_clock> last_fw_request = std::chrono::steady_clock::now();

        while (update_thread_run_) {
            auto time = (const struct timespec) {0, state_request_millis * 1000000L};
            nanosleep(&time, nullptr);

            std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
            VESC_CONNECTION_STATE state;
            {
                std::unique_lock<std::mutex> lk(status_mutex_);
                state = status_.connection_state;
            }

            if (WAITING_FOR_FW == state) {
                if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_fw_request).count() >
                    1000) {
                    last_fw_request = now;
                    requestFWVersion();
                }
                continue;
            } else if(status_.connection_state == CONNECTED || status_.connection_state == CONNECTED_INCOMPATIBLE_FW) {
                requestState();
            }
        }
        return nullptr;
    }

    void *VescInterface::rx_thread() {
        Buffer buffer;
        buffer.reserve(4096);

        {
            std::unique_lock<std::mutex> lk(status_mutex_);
            status_ = {0};
            status_.connection_state = DISCONNECTED;
        }
        while (rx_thread_run_) {
            // Check, if the serial port is connected. If not, connect to it.
            if (!serial_.isOpen()) {
                {
                    std::unique_lock<std::mutex> lk(status_mutex_);
                    status_ = {0};
                    status_.connection_state = DISCONNECTED;
                }
                try {
                    serial_.setPort(port_);
                    serial_.open();
                } catch (std::exception &e) {
                    // retry later
                    sleep(1);
                    continue;
                }
                {
                    std::unique_lock<std::mutex> lk(status_mutex_);
                    status_.connection_state = VESC_CONNECTION_STATE::WAITING_FOR_FW;
                }
            }

            int bytes_needed = VescFrame::VESC_MIN_FRAME_SIZE;
            if (!buffer.empty()) {
                // search buffer for valid packet(s)
                Buffer::iterator iter(buffer.begin());
                Buffer::iterator iter_begin(buffer.begin());
                while (iter != buffer.end()) {
                    // check if valid start-of-frame character
                    if (VescFrame::VESC_SOF_VAL_SMALL_FRAME == *iter ||
                        VescFrame::VESC_SOF_VAL_LARGE_FRAME == *iter) {
                        // good start, now attempt to create packet
                        std::string error;
                        VescPacketConstPtr packet = VescPacketFactory::createPacket(iter, buffer.end(),
                                                                                    &bytes_needed,
                                                                                    &error);
                        if (packet) {
                            // good packet, check if we skipped any data
                            if (std::distance(iter_begin, iter) > 0) {
                                std::ostringstream ss;
                                ss << "Out-of-sync with VESC, unknown data leading valid frame. Discarding "
                                   << std::distance(iter_begin, iter) << " bytes.";
                                error_handler_(ss.str());
                            }
                            // call packet handler

                            handle_packet(packet);
                            // update state
                            iter = iter + packet->getFrame().size();
                            iter_begin = iter;
                            // continue to look for another frame in buffer
                            continue;
                        } else if (bytes_needed > 0) {
                            // need more data, break out of while loop
                            break;  // for (iter_sof...
                        } else {
                            // else, this was not a packet, move on to next byte
                            error_handler_(error);
                        }
                    }

                    iter++;
                }

                // if iter is at the end of the buffer, more bytes are needed
                if (iter == buffer.end())
                    bytes_needed = VescFrame::VESC_MIN_FRAME_SIZE;

                // erase "used" buffer
                if (std::distance(iter_begin, iter) > 0) {
                    std::ostringstream ss;
                    ss << "Out-of-sync with VESC, discarding " << std::distance(iter_begin, iter) << " bytes.";
                    error_handler_(ss.str());
                }
                buffer.erase(buffer.begin(), iter);
            }

            // attempt to read at least bytes_needed bytes from the serial port
            int bytes_to_read = std::max(bytes_needed, std::min(4096, static_cast<int>(serial_.available())));
            try {
                int bytes_read = serial_.read(buffer, bytes_to_read);
                if (bytes_needed > 0 && 0 == bytes_read && !buffer.empty()) {
                    error_handler_("Possibly out-of-sync with VESC, read timout in the middle of a frame.");
                }
            } catch (std::exception &e) {
                error_handler_("error during serial read. reconnecting.");
                {
                    std::unique_lock<std::mutex> lk(status_mutex_);
                    status_.connection_state = VESC_CONNECTION_STATE::DISCONNECTED;
                }
                serial_.close();
            }
        }


        serial_.close();

        return nullptr;
    }

    void VescInterface::handle_packet(VescPacketConstPtr packet) {
        // Only update the state if connection state is connected
        if ((status_.connection_state == CONNECTED || status_.connection_state == CONNECTED_INCOMPATIBLE_FW) &&
            packet->getName() == "Values") {
            std::lock_guard<std::mutex> lk(status_mutex_);
            std::shared_ptr<VescPacketValues const> values = std::dynamic_pointer_cast<VescPacketValues const>(
                    packet);

            status_.seq++;
            status_.voltage_input = values->getInputVoltage();
            status_.temperature_pcb = values->getMosTemp();
            status_.temperature_motor = values->getMotorTemp();
            status_.current_motor = values->getMotorCurrent();
            status_.current_input = values->getInputCurrent();
            status_.speed_erpm = values->getVelocityERPM();
            status_.duty_cycle = values->getDuty();
            status_.charge_drawn = values->getConsumedCharge();
            status_.charge_regen = values->getInputCharge();
            status_.energy_drawn = values->getConsumedPower();
            status_.energy_regen = values->getInputPower();
            status_.displacement = values->getPosition();
            status_.distance_traveled = values->getDisplacement();
            status_.fault_code = values->getFaultCode();
            status_.tacho = values->getPosition();
            status_.tacho_absolute = values->getDisplacement();
            status_.direction = values->getVelocityERPM() < 0;
            status_cv_.notify_all();
        } else if (packet->getName() == "FWVersion") {
            std::lock_guard<std::mutex> lk(status_mutex_);
            std::shared_ptr<VescPacketFWVersion const> fw_version =
                    std::dynamic_pointer_cast<VescPacketFWVersion const>(packet);

            status_.seq++;
            status_.fw_version_major = fw_version->fwMajor();
            status_.fw_version_minor = fw_version->fwMinor();

            // check for fully compatible FW here
            if (status_.fw_version_major == 5 && status_.fw_version_minor == 3) {
                status_.connection_state = CONNECTED;
            } else {
                status_.connection_state = CONNECTED_INCOMPATIBLE_FW;
            }

            status_cv_.notify_all();
        }
    }


    bool VescInterface::send(const VescPacket &packet) {
        std::unique_lock<std::mutex> lk(serial_tx_mutex_);
        if (!serial_.isOpen()) {
            return false;
        }
        size_t written = serial_.write(packet.getFrame());
        if (written != packet.getFrame().size()) {
            return false;
        }
        return true;
    }

    void VescInterface::requestFWVersion() {
        send(VescPacketRequestFWVersion());
    }

    void VescInterface::requestState() {
        send(VescPacketRequestValues());
    }

    void VescInterface::setDutyCycle(double duty_cycle) {
        send(VescPacketSetDuty(duty_cycle));
    }

    void VescInterface::setCurrent(double current) {
        send(VescPacketSetCurrent(current));
    }

    void VescInterface::setBrake(double brake) {
        send(VescPacketSetCurrentBrake(brake));
    }

    void VescInterface::setSpeed(double speed) {
        send(VescPacketSetVelocityERPM(speed));
    }

    void VescInterface::setPosition(double position) {
        send(VescPacketSetPos(position));
    }

    void VescInterface::start(const std::string &port) {
        port_ = port;
        // start up a monitoring thread
        rx_thread_run_ = true;
        update_thread_run_ = true;
        pthread_create(&rx_thread_handle_, NULL, &VescInterface::rx_thread_helper, this);
        pthread_create(&update_thread_handle_, NULL, &VescInterface::update_thread_helper, this);
    }

    void VescInterface::stop() {
        // stops the motor
        setDutyCycle(0.0);

        // tell the io thread to stop
        rx_thread_run_ = false;
        update_thread_run_ = false;

        // wait for io thread to actually exit
        pthread_join(rx_thread_handle_, nullptr);
        pthread_join(update_thread_handle_, nullptr);
    }

    void VescInterface::get_status(VescStatusStruct *status) {
        std::unique_lock<std::mutex> lk(status_mutex_);
        *status = status_;
    }

    void VescInterface::wait_for_status(VescStatusStruct *status) {
        std::unique_lock<std::mutex> lk(status_mutex_);
        // wait for new data
        status_cv_.wait(lk);
        *status = status_;
    }


}  // namespace xesc_2040_driver
