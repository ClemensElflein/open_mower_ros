#ifndef XESC2040_DRIVER_XESC2040_INTERFACE_H_
#define XESC2040_DRIVER_XESC2040_INTERFACE_H_

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
#include "COBS.h"
#include "xesc_2040_driver/xesc_2040_datatypes.h"


namespace xesc_2040_driver {

    enum XESC2040_CONNECTION_STATE {
        DISCONNECTED = 0,
        CONNECTED = 3,
    };

    struct Xesc2040StatusStruct {
        uint32_t seq;
        uint8_t fw_version_major;
        uint8_t fw_version_minor;
        XESC2040_CONNECTION_STATE connection_state;
        double voltage_input;       // input voltage (volt)
        double temperature_pcb;     // temperature of printed circuit board (degrees Celsius)
        double temperature_motor;   // temperature of printed circuit board (degrees Celsius)
        double current_input;       // input current (ampere)
        double duty_cycle;          // duty cycle (0 to 1)
        uint32_t tacho;
        uint32_t tacho_absolute;    // wheel ticks absolute
        bool direction;             // direction CW/CCW
        int32_t fault_code;
    };

    class Xesc2040Interface : private boost::noncopyable {
    public:
        typedef std::function<void(const std::string &)> ErrorHandlerFunction;

        Xesc2040Interface(const ErrorHandlerFunction &error_handler);

        ~Xesc2040Interface();

        void setDutyCycle(double duty_cycle);

        void start(const std::string &port);

        void stop();

        void get_status(Xesc2040StatusStruct *status);

        void wait_for_status(Xesc2040StatusStruct *status);

        void update_settings(uint8_t *hall_table,
                             float motor_current_limit,
                             float acceleration,
                             bool has_motor_temp,
                             float min_motor_temp,
                             float max_motor_temp,
                             float min_pcb_temp,
                             float max_pcb_temp);

    private:

        bool send(uint8_t *packet, size_t size);

        void send_settings();

        static void *rx_thread_helper(void *context) {
            return ((Xesc2040Interface *) context)->rx_thread();
        }


        void *rx_thread();


        void handle_packet(Xesc2040StatusPacket *packet);


        pthread_t rx_thread_handle_;
        bool rx_thread_run_;


        ErrorHandlerFunction error_handler_;
        serial::Serial serial_;
        std::string port_;
        std::mutex status_mutex_;
        // since multiple threads will call the send() function, we need a mutex.
        std::mutex serial_tx_mutex_;
        std::condition_variable status_cv_;
        struct Xesc2040StatusStruct status_;
        struct Xesc2040SettingsPacket settings_;
        bool settings_valid;

        uint8_t tx_buffer[1000];
        boost::crc_ccitt_type tx_crc;
    };

}  // namespace xesc_2040_driver

#endif
