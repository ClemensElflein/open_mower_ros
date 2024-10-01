#ifndef XESCYFR4_DRIVER_XESCYFR4_INTERFACE_H_
#define XESCYFR4_DRIVER_XESCYFR4_INTERFACE_H_

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
#include "xesc_yfr4_driver/xesc_yfr4_datatypes.h"


namespace xesc_yfr4_driver {

    enum XESCYFR4_CONNECTION_STATE {
        DISCONNECTED = 0,
        CONNECTED = 3,
    };

    struct XescYFR4StatusStruct {
        uint32_t seq;
        uint8_t fw_version_major;
        uint8_t fw_version_minor;
        XESCYFR4_CONNECTION_STATE connection_state;
        double temperature_pcb;     // temperature of printed circuit board (degrees Celsius)
        double current_input;       // input current (ampere)
        double duty_cycle;          // duty cycle (0 to 1)
        bool direction;             // direction CW/CCW
        uint32_t tacho;
        uint32_t tacho_absolute;    // wheel ticks absolute
        uint16_t rpm;               // revolutions per minute (of the axis/shaft)
        int32_t fault_code;
    };

    class XescYFR4Interface : private boost::noncopyable {
    public:
        typedef std::function<void(const std::string &)> ErrorHandlerFunction;

        XescYFR4Interface(const ErrorHandlerFunction &error_handler);

        ~XescYFR4Interface();

        void setDutyCycle(double duty_cycle);

        void start(const std::string &port);

        void stop();

        void get_status(XescYFR4StatusStruct *status);

        void wait_for_status(XescYFR4StatusStruct *status);

        void update_settings(float motor_current_limit, float min_pcb_temp, float max_pcb_temp);

    private:

        bool send(uint8_t *packet, size_t size);

        void send_settings();

        static void *rx_thread_helper(void *context) {
            return ((XescYFR4Interface *) context)->rx_thread();
        }


        void *rx_thread();


        void handle_packet(XescYFR4StatusPacket *packet);


        pthread_t rx_thread_handle_;
        bool rx_thread_run_;


        ErrorHandlerFunction error_handler_;
        serial::Serial serial_;
        std::string port_;
        std::mutex status_mutex_;
        // since multiple threads will call the send() function, we need a mutex.
        std::mutex serial_tx_mutex_;
        std::condition_variable status_cv_;
        struct XescYFR4StatusStruct status_;
        struct XescYFR4SettingsPacket settings_;
        bool settings_valid;

        uint8_t tx_buffer[1000];
        boost::crc_ccitt_type tx_crc;
    };

}  // namespace xesc_yfr4_driver

#endif
