/*********************************************************************
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
 ********************************************************************/

#ifndef VESC_DRIVER_DATA_MAP_H_
#define VESC_DRIVER_DATA_MAP_H_

/**
 * @brief ID of communication commands
 **/
enum COMM_PACKET_ID
{
  COMM_FW_VERSION = 0,
  COMM_JUMP_TO_BOOTLOADER,
  COMM_ERASE_NEW_APP,
  COMM_WRITE_NEW_APP_DATA,
  COMM_GET_VALUES,
  COMM_SET_DUTY,
  COMM_SET_CURRENT,
  COMM_SET_CURRENT_BRAKE,
  COMM_SET_ERPM,
  COMM_SET_POS,
  COMM_SET_HANDBRAKE,
  COMM_SET_DETECT,
  COMM_SET_SERVO_POS,
  COMM_SET_MCCONF,
  COMM_GET_MCCONF,
  COMM_GET_MCCONF_DEFAULT,
  COMM_SET_APPCONF,
  COMM_GET_APPCONF,
  COMM_GET_APPCONF_DEFAULT,
  COMM_SAMPLE_PRINT,
  COMM_TERMINAL_CMD,
  COMM_PRINT,
  COMM_ROTOR_POSITION,
  COMM_EXPERIMENT_SAMPLE,
  COMM_DETECT_MOTOR_PARAM,
  COMM_DETECT_MOTOR_R_L,
  COMM_DETECT_MOTOR_FLUX_LINKAGE,
  COMM_DETECT_ENCODER,
  COMM_DETECT_HALL_FOC,
  COMM_REBOOT,
  COMM_ALIVE,
  COMM_GET_DECODED_PPM,
  COMM_GET_DECODED_ADC,
  COMM_GET_DECODED_CHUK,
  COMM_FORWARD_CAN,
  COMM_SET_CHUCK_DATA,
  COMM_CUSTOM_APP_DATA,
  COMM_NRF_START_PAIRING
};

/**
 * @brief Map of return packets
 **/
enum PACKET_MAP
{
  TEMP_MOS = 1,
  TEMP_MOTOR = 3,
  CURRENT_MOTOR = 5,
  CURRENT_IN = 9,
  ID = 13,
  IQ = 17,
  DUTY_NOW = 21,
  ERPM = 23,
  VOLTAGE_IN = 27,
  AMP_HOURS = 29,
  AMP_HOURS_CHARGED = 33,
  WATT_HOURS = 37,
  WATT_HOURS_CHARGED = 41,
  TACHOMETER = 45,
  TACHOMETER_ABS = 49,
  FAULT_CODE = 53,
};

#endif  // VESC_DRIVER_DATA_MAP_H_
