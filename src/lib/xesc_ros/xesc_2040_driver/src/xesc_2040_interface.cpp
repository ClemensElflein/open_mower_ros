#include "xesc_2040_driver/xesc_2040_interface.h"

namespace xesc_2040_driver {

    Xesc2040Interface::Xesc2040Interface(const ErrorHandlerFunction &error_handler)
            : serial_(std::string(), 115200, serial::Timeout::simpleTimeout(100), serial::eightbits,
                      serial::parity_none,
                      serial::stopbits_one, serial::flowcontrol_none) {

        error_handler_ = error_handler;
        settings_valid = false;
    }

    Xesc2040Interface::~Xesc2040Interface() {
        stop();
    }


    void *Xesc2040Interface::rx_thread() {
        COBS cobs;
        boost::crc_ccitt_type crc;

        size_t buffer_size = 1000;// cobs.getEncodedBufferSize(sizeof(Xesc2040StatusPacket));
        uint8_t buffer[buffer_size];
        uint8_t buffer_decoded[buffer_size];
        size_t read = 0;

        {
            std::unique_lock<std::mutex> lk(status_mutex_);
            status_ = {0};
            status_.connection_state = XESC2040_CONNECTION_STATE::DISCONNECTED;
        }
        while (rx_thread_run_) {
            if (!serial_.isOpen()) {
                try {
                    status_.connection_state = XESC2040_CONNECTION_STATE::DISCONNECTED;

                    serial_.setPort(port_);
                    serial_.setBaudrate(115200);
                    auto to = serial::Timeout::simpleTimeout(100);
                    serial_.setTimeout(to);
                    serial_.open();
                    // wait for controller to boot
                    sleep(1);
                    status_.connection_state = XESC2040_CONNECTION_STATE::CONNECTED;
                    send_settings();
                } catch (std::exception &e) {
                    sleep(1);
                    std::ostringstream ss;
                    ss << "Error connecting to xESC on Port:" << serial_.getPort();
                    error_handler_(ss.str());
                }
            }
            size_t bytes_read = 0;
            try {
                bytes_read = serial_.read(buffer + read, 1);
            } catch (std::exception &e) {
                std::ostringstream ss;
                ss << "Error reading serial_port. Closing Connection. Port:" << serial_.getPort();
                error_handler_(ss.str());
                serial_.close();
                sleep(1);
            }
            if (read + bytes_read >= buffer_size) {
                read = 0;
                bytes_read = 0;
                std::ostringstream ss;
                ss << "Prevented buffer overflow. There is a problem with the serial comms. Port:" << serial_.getPort();
                error_handler_(ss.str());
            }
            if (bytes_read) {
                if (buffer[read] == 0) {
                    // end of packet found
                    size_t data_size = cobs.decode(buffer, read, buffer_decoded);

                    // first, check the CRC
                    if (data_size < 3) {
                        // We don't even have one byte of data
                        // (type + crc = 3 bytes already)
                        std::ostringstream ss;
                        ss << "Got empty packet from xESC. Port:" << serial_.getPort();
                        error_handler_(ss.str());
                    } else {
                        // We have at least 1 byte of data, check the CRC
                        crc.reset();
                        // We start at the second byte (ignore the type) and process (data_size- byte for type - 2 bytes for CRC) bytes.
                        crc.process_bytes(buffer_decoded, data_size - 2);
                        uint16_t checksum = crc.checksum();
                        uint16_t received_checksum = *(uint16_t *) (buffer_decoded + data_size - 2);
                        if (checksum == received_checksum) {
                            // Packet checksum is OK, process it
                            switch (buffer_decoded[0]) {

                                case XESC2040_MSG_TYPE_STATUS: {
                                    if (data_size == sizeof(struct Xesc2040StatusPacket)) {
                                        handle_packet((Xesc2040StatusPacket *) buffer_decoded);
                                    } else {
                                        std::ostringstream ss;
                                        ss << "Got packet with wrong size on port:" << serial_.getPort() << ". id was:"
                                           << (int) buffer_decoded[0];
                                        error_handler_(ss.str());
                                    }
                                }

                                    break;

                                default:
                                    std::ostringstream ss;
                                    ss << "Got unknown valid packet from xESC on Port:" << serial_.getPort()
                                       << ". id was:" << (int) buffer_decoded[0];
                                    error_handler_(ss.str());
                                    break;
                            }
                        } else {
                            std::ostringstream ss;
                            ss << "Got invalid checksum from xESC on Port:" << serial_.getPort();
                            error_handler_(ss.str());
                        }

                    }

                    read = 0;
                } else {
                    read += bytes_read;
                }
            }
        }


        serial_.close();

        return nullptr;
    }

    void Xesc2040Interface::handle_packet(Xesc2040StatusPacket *packet) {
        // Only update the state if connection state is connected
        if (status_.connection_state == CONNECTED) {
            std::lock_guard<std::mutex> lk(status_mutex_);
            status_.seq = packet->seq;
            status_.fw_version_major = packet->fw_version_major;
            status_.fw_version_minor = packet->fw_version_minor;
            status_.voltage_input = packet->voltage_input;
            status_.temperature_pcb = packet->temperature_pcb;
            status_.temperature_motor = packet->temperature_motor;
            status_.current_input = packet->current_input;
            status_.duty_cycle = packet->duty_cycle;
            status_.tacho = packet->tacho;
            status_.tacho_absolute = packet->tacho_absolute;
            status_.direction = packet->direction;
            status_.fault_code = packet->fault_code;

            // If unconfigured, send settings
            if(packet->fault_code & FAULT_UNINITIALIZED) {
                send_settings();
            }

            status_cv_.notify_all();
        } else {
            std::ostringstream ss;
            ss << "Got packet with status != connected on port:" << serial_.getPort();
            error_handler_(ss.str());
        }
    }


    bool Xesc2040Interface::send(uint8_t *packet, size_t length) {
        std::unique_lock<std::mutex> lk(serial_tx_mutex_);
        if (!serial_.isOpen()) {
            return false;
        }

        tx_crc.reset();
        tx_crc.process_bytes(packet, length - 2);
        uint16_t checksum = tx_crc.checksum();
        *(uint16_t *) (packet + length - 2) = checksum;

        size_t encoded_size = COBS::encode(packet, length, tx_buffer);
        tx_buffer[encoded_size] = 0;
        encoded_size++;
        size_t written = serial_.write(tx_buffer, encoded_size);
        if (written != encoded_size) {
            return false;
        }
        return true;
    }

    void Xesc2040Interface::setDutyCycle(double duty_cycle) {
        Xesc2040ControlPacket controlPacket = {.message_type = XESC2040_MSG_TYPE_CONTROL, .duty_cycle = duty_cycle, .crc=0};
        send(reinterpret_cast<uint8_t *>(&controlPacket), sizeof(controlPacket));
    }

    void Xesc2040Interface::start(const std::string &port) {
        port_ = port;
        // start up a monitoring thread
        rx_thread_run_ = true;
        pthread_create(&rx_thread_handle_, NULL, &Xesc2040Interface::rx_thread_helper, this);
    }

    void Xesc2040Interface::stop() {
        // stops the motor
        setDutyCycle(0.0);

        // tell the io thread to stop
        rx_thread_run_ = false;

        // wait for io thread to actually exit
        pthread_join(rx_thread_handle_, nullptr);
    }

    void Xesc2040Interface::get_status(Xesc2040StatusStruct *status) {
        std::unique_lock<std::mutex> lk(status_mutex_);
        *status = status_;
    }

    void Xesc2040Interface::wait_for_status(Xesc2040StatusStruct *status) {
        std::unique_lock<std::mutex> lk(status_mutex_);
        // wait for new data
        status_cv_.wait(lk);
        *status = status_;
    }

    void Xesc2040Interface::send_settings() {
        if (!settings_valid) {
            // don't send them, error on the console
            error_handler_("Error sending xESC settings: Settings invalid. Call update_settings first!");
            return;
        }
        settings_.message_type = XESC2040_MSG_TYPE_SETTINGS;
        send(reinterpret_cast<uint8_t *>(&settings_), sizeof(settings_));
    }

    void Xesc2040Interface::update_settings(uint8_t *hall_table,
                                            float motor_current_limit,
                                            float acceleration,
                                            bool has_motor_temp,
                                            float min_motor_temp,
                                            float max_motor_temp,
                                            float min_pcb_temp,
                                            float max_pcb_temp) {
        for (int i = 0; i < 8; i++) {
            settings_.hall_table[i] = hall_table[i];
        }
        settings_.motor_current_limit = motor_current_limit;
        settings_.acceleration = acceleration;
        settings_.has_motor_temp = has_motor_temp;
        settings_.min_motor_temp = min_motor_temp;
        settings_.max_motor_temp = max_motor_temp;
        settings_.min_pcb_temp = min_pcb_temp;
        settings_.max_pcb_temp = max_pcb_temp;
        settings_valid = true;
        send_settings();
    }


}  // namespace xesc_2040_driver
