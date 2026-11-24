#ifndef ESCSTATUSCOMPAT_H
#define ESCSTATUSCOMPAT_H

#include <mower_msgs/ESCStatus.h>

#include <cstdint>

/**
 * @brief Compile-time check that a service enum still matches the mower_msgs constants.
 *
 * The ESC status values exist three times: in the service definition (the generated enums),
 * in mower_msgs/ESCStatus (ROS message constants) and in the firmware's MotorDriver. They must
 * move together; this turns a silent drift into a build error.
 */
template <typename T>
constexpr bool EscStatusMatchesMowerMsgs() {
  return static_cast<uint8_t>(T::DISCONNECTED) == mower_msgs::ESCStatus::ESC_STATUS_DISCONNECTED &&
         static_cast<uint8_t>(T::ERROR) == mower_msgs::ESCStatus::ESC_STATUS_ERROR &&
         static_cast<uint8_t>(T::STALLED) == mower_msgs::ESCStatus::ESC_STATUS_STALLED &&
         static_cast<uint8_t>(T::OK) == mower_msgs::ESCStatus::ESC_STATUS_OK &&
         static_cast<uint8_t>(T::RUNNING) == mower_msgs::ESCStatus::ESC_STATUS_RUNNING;
}

#endif  // ESCSTATUSCOMPAT_H
