// Created by Clemens Elflein on 3/07/22.
// Copyright (c) 2022 Clemens Elflein and OpenMower contributors. All rights reserved.
//
// This file is part of OpenMower.
//
// OpenMower is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
// License as published by the Free Software Foundation, version 3 of the License.
//
// OpenMower is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with OpenMower. If not, see
// <https://www.gnu.org/licenses/>.
//

#ifndef _DATATYPES_H
#define _DATATYPES_H

#include <stdint.h>

#define PACKET_ID_LL_STATUS 1
#define PACKET_ID_LL_IMU 2
#define PACKET_ID_LL_UI_EVENT 3
#define PACKET_ID_LL_HIGH_LEVEL_CONFIG_REQ 0x11  // ll_high_level_config and request config from receiver
#define PACKET_ID_LL_HIGH_LEVEL_CONFIG_RSP 0x12  // ll_high_level_config response
#define PACKET_ID_LL_HEARTBEAT 0x42
#define PACKET_ID_LL_HIGH_LEVEL_STATE 0x43

// clang-format off
#define LL_EMERGENCY_BIT_LATCH (1 << 0)  // Any emergency latch
#define LL_EMERGENCY_BIT_STOP  (1 << 1)  // Stop
#define LL_EMERGENCY_BIT_LIFT  (1 << 2)  // Lift (or tilt)

// CoverUI will stay with the old emergency_bitmask definition
#define LL_EMERGENCY_BIT_CU_LATCH (1 << 0)  // Any emergency latch
#define LL_EMERGENCY_BIT_CU_STOP1 (1 << 1)  // Stop1
#define LL_EMERGENCY_BIT_CU_STOP2 (1 << 2)  // Stop2
#define LL_EMERGENCY_BIT_CU_LIFT  (1 << 3)  // LIFT | LIFTX (up to CoverUI FW 2.0x)
#define LL_EMERGENCY_BIT_CU_BUMP  (1 << 4)  // LBUMP | RBUMP (up to CoverUI FW 2.0x)
#define LL_EMERGENCY_BIT_CU_LIFTX (1 << 5)  // CoverUI-LIFTX (as of CoverUI FW 2.1x)
#define LL_EMERGENCY_BIT_CU_RBUMP (1 << 6)  // CoverUI-RBUMP (as of CoverUI FW 2.1x)

#define LL_STATUS_BIT_INITIALIZED (1 << 0)
#define LL_STATUS_BIT_RASPI_POWER (1 << 1)
#define LL_STATUS_BIT_CHARGING    (1 << 2)
#define LL_STATUS_BIT_FREE        (1 << 3)
#define LL_STATUS_BIT_RAIN        (1 << 4)
#define LL_STATUS_BIT_SOUND_AVAIL (1 << 5)
#define LL_STATUS_BIT_SOUND_BUSY  (1 << 6)
#define LL_STATUS_BIT_UI_AVAIL    (1 << 7)
// clang-format on

#pragma pack(push, 1)
struct ll_status {
  // Type of this message. Has to be PACKET_ID_LL_STATUS.
  uint8_t type;
  // Bitmask for rain, sound, powers etc
  // Bit 0: Initialized (i.e. setup() was a success). If this is 0, all other bits are meaningless.
  // Bit 1: Raspberry Power
  // Bit 2: GPS Power
  // Bit 3: ESC Power
  // Bit 4: Rain detected
  // Bit 5: Sound available
  // Bit 6: Sound busy
  // Bit 7: UI Board available
  uint8_t status_bitmask;
  // USS range in m
  float uss_ranges_m[5];
  // Emergency bitmask:
  // Bit 0: Emergency latch
  // Bit 1: Emergency/Lift (or tilt)
  // Bit 2: Emergency/Stop
  uint8_t emergency_bitmask;
  // Charge voltage
  float v_charge;
  // System voltage
  float v_system;
  // Charge current
  float charging_current;
  uint8_t batt_percentage;
  uint16_t crc;
} __attribute__((packed));
#pragma pack(pop)

#pragma pack(push, 1)
struct ll_imu {
  // Type of this message. Has to be PACKET_ID_LL_IMU.
  uint8_t type;
  // Time since last message in milliseconds.
  uint16_t dt_millis;
  // Acceleration[m^s2], Gyro[rad/s] and magnetic field[uT]
  float acceleration_mss[3];
  float gyro_rads[3];
  float mag_uT[3];
  uint16_t crc;
} __attribute__((packed));
#pragma pack(pop)

#pragma pack(push, 1)
struct ll_heartbeat {
  // Type of this message. Has to be PACKET_ID_LL_HEARTBEAT.
  uint8_t type;
  // True, if emergency should be engaged (e.g. navigation error, ...)
  // False to not change the latch
  uint8_t emergency_requested;
  // True, if emergency condition should be reset
  // False to not change the latch
  uint8_t emergency_release_requested;
  uint16_t crc;
} __attribute__((packed));
#pragma pack(pop)

#pragma pack(push, 1)
struct ll_ui_event {
  // Type of this message. Has to be PACKET_ID_LL_UI_EVENT
  uint8_t type;
  uint8_t button_id;
  uint8_t press_duration;  // 0 for single press, 1 for long, 2 for very long press
  uint16_t crc;
} __attribute__((packed));
#pragma pack(pop)

#pragma pack(push, 1)
struct ll_high_level_state {
  // Type of this message. Has to be PACKET_ID_LL_HIGH_LEVEL_STATE
  uint8_t type;
  uint8_t current_mode;  // see HighLevelMode
  uint8_t gps_quality;   // GPS quality in percent (0-100)
  uint16_t crc;
} __attribute__((packed));
#pragma pack(pop)

enum class OptionState : unsigned int { OFF = 0, ON, UNDEFINED };

#pragma pack(push, 1)
struct ConfigOptions {
  OptionState dfp_is_5v : 2;
  OptionState background_sounds : 2;
  OptionState ignore_charging_current : 2;
  // Need to block/waster the bits now, to be prepared for future enhancements
  OptionState reserved_for_future_use1 : 2;
  OptionState reserved_for_future_use2 : 2;
  OptionState reserved_for_future_use3 : 2;
  OptionState reserved_for_future_use4 : 2;
  OptionState reserved_for_future_use5 : 2;
} __attribute__((packed));
#pragma pack(pop)
static_assert(sizeof(ConfigOptions) == 2, "Changing size of ConfigOption != 2 will break packet compatibilty");

typedef char iso639_1[2];  // Two char ISO 639-1 language code

enum class HallMode : unsigned int {
  OFF = 0,
  LIFT_TILT,  // Wheel lifted and wheels tilted functionality
  STOP,       // Stop mower
  UNDEFINED   // This is used by foreign side to inform that it doesn't has a configuration for this sensor
};

#pragma pack(push, 1)
struct HallConfig {
  HallConfig(HallMode t_mode = HallMode::UNDEFINED, bool t_active_low = false)
      : mode(t_mode), active_low(t_active_low){};

  HallMode mode : 3;  // 1 bit reserved
  bool active_low : 1;
} __attribute__((packed));
#pragma pack(pop)

#define MAX_HALL_INPUTS 10  // How much Hall-inputs we do support. 4 * OM + 6 * Stock-CoverUI

// LL/HL config packet, bi-directional, flexible-length
#pragma pack(push, 1)
struct ll_high_level_config {
  // ATTENTION: This is a flexible length struct. It is allowed to grow independently to HL without loosing
  // compatibility, but never change or restructure already published member, except you really know their consequences.

  // uint8_t type; Just for illustration. Get set later in wire buffer with type PACKET_ID_LL_HIGH_LEVEL_CONFIG_*

  // clang-format off
  ConfigOptions options = {.dfp_is_5v = OptionState::OFF, .background_sounds = OptionState::OFF, .ignore_charging_current = OptionState::OFF};
  uint16_t rain_threshold = 0xffff;          // If (stock CoverUI) rain value < rain_threshold then it rains
  float v_charge_cutoff = -1;                // Protective max. charging voltage before charging get switched off (-1 = unknown)
  float i_charge_cutoff = -1;                // Protective max. charging current before charging get switched off (-1 = unknown)
  float v_battery_cutoff = -1;               // Protective max. battery voltage before charging get switched off (-1 = unknown)
  float v_battery_empty = -1;                // Empty battery voltage used for % calc of capacity (-1 = unknown)
  float v_battery_full = -1;                 // Full battery voltage used for % calc of capacity (-1 = unknown)
  uint16_t lift_period = 0xffff;             // Period (ms) for >=2 wheels to be lifted in order to count as emergency (0 = disable, 0xFFFF = unknown)
  uint16_t tilt_period = 0xffff;             // Period (ms) for a single wheel to be lifted in order to count as emergency (0 = disable, 0xFFFF = unknown)
  uint8_t shutdown_esc_max_pitch = 0xff;     // Do not shutdown ESCs if absolute pitch angle is greater than this (0 = disable, 0xff = unknown) (to be implemented, see OpenMower PR #97)
  iso639_1 language = {'e', 'n'};            // ISO 639-1 (2-char) language code (en, de, ...)
  uint8_t volume = 0xff;                     // Volume (0-100%) feedback (if directly changed i.e. via CoverUI) (0xff = do not change)
  HallConfig hall_configs[MAX_HALL_INPUTS];  // Set all to UNDEFINED
  // INFO: Before adding a new member here: Decide if and how much hall_configs spares do we like to have

  // uint16_t crc;  Just for illustration, that it get appended later within the wire buffer
  // clang-format on
} __attribute__((packed));
#pragma pack(pop)

#endif
