#ifndef SRC_XESC_YFR4_DATATYPES_H
#define SRC_XESC_YFR4_DATATYPES_H

#include <cstdint>

#define XESCYFR4_MSG_TYPE_STATUS 1
#define XESCYFR4_MSG_TYPE_CONTROL 2
#define XESCYFR4_MSG_TYPE_SETTINGS 3

#define FAULT_UNINITIALIZED  (1u << 0)
#define FAULT_WATCHDOG       (1u << 1)
#define FAULT_UNDERVOLTAGE   (1u << 2)
#define FAULT_OVERVOLTAGE    (1u << 3)
#define FAULT_OVERCURRENT    (1u << 4)
#define FAULT_OVERTEMP_MOTOR (1u << 5)
#define FAULT_OVERTEMP_PCB   (1u << 6)
#define FAULT_INVALID_HALL   (1u << 7)

#pragma pack(push, 1)
struct XescYFR4StatusPacket {
    uint8_t message_type;
    uint32_t seq;
    uint8_t fw_version_major;
    uint8_t fw_version_minor;
    double temperature_pcb;     // temperature of printed circuit board (degrees Celsius)
    double current_input;       // input current (ampere)
    double duty_cycle;          // duty cycle (0 to 1)
    uint32_t tacho;
    uint32_t tacho_absolute;    // wheel ticks absolute
    bool direction;             // direction CW/CCW
    int32_t fault_code;
    uint16_t crc;
} __attribute__((packed));
#pragma pack(pop)

#pragma pack(push, 1)
struct XescYFR4ControlPacket {
    uint8_t message_type;
    double duty_cycle;           // duty cycle (0 to 1)
    uint16_t crc;
} __attribute__((packed));
#pragma pack(pop)

#pragma pack(push, 1)
struct XescYFR4SettingsPacket {
    uint8_t message_type;
    float motor_current_limit;
    float min_pcb_temp;
    float max_pcb_temp;
    uint16_t crc;
} __attribute__((packed));
#pragma pack(pop)

#endif  // SRC_XESC_YFR4_DATATYPES_H
