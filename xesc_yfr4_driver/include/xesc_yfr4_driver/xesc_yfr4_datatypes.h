#ifndef SRC_XESC_YFR4_DATATYPES_H
#define SRC_XESC_YFR4_DATATYPES_H

#include <cstdint>

#define XESCYFR4_MSG_TYPE_STATUS 1
#define XESCYFR4_MSG_TYPE_CONTROL 2
#define XESCYFR4_MSG_TYPE_SETTINGS 3

#define FAULT_UNINITIALIZED  0b1
#define FAULT_WATCHDOG       0b10
#define FAULT_UNDERVOLTAGE   0b100
#define FAULT_OVERVOLTAGE    0b1000
#define FAULT_OVERCURRENT    0b10000
#define FAULT_OVERTEMP_MOTOR 0b100000
#define FAULT_OVERTEMP_PCB   0b1000000
#define FAULT_INVALID_HALL   0b10000000
#define FAULT_INTERNAL_ERROR 0b100000000
#define FAULT_OPEN_LOAD      0b1000000000

#pragma pack(push, 1)
struct XescYFR4StatusPacket {
    uint8_t message_type;
    uint32_t seq;
    uint8_t fw_version_major;
    uint8_t fw_version_minor;
    double temperature_pcb;     // temperature of printed circuit board (degrees Celsius)
    double current_input;       // input current (ampere)
    double duty_cycle;          // duty cycle (0 to 1)
    bool direction;             // direction CW/CCW
    uint32_t tacho;             // wheel ticks
    uint32_t tacho_absolute;    // wheel ticks absolute
    uint16_t rpm;               // revolutions per minute (of the axis/shaft)
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
